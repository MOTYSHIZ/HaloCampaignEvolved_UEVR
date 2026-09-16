// core reload engine (runs while reloadvr or slidevr is on): the weapon's own magazine, the well marker, the reload animation hold, the Wwise reload sound and the ammo probes.
// Textual fragment, included by core/reload/ReloadEngine.cpp in its anonymous namespace, after the engine's reload state. Moved verbatim; not compiled on its own.
// ---- THE WEAPON'S OWN MAGAZINE -----------------------------------------------------------------
// The gun ships its magazine as a component on the weapon actor, so during MAG_OUT it can be
// genuinely removed instead of the rifle pretending nothing happened ("the magazine still appears
// to be in the assault rifle" -- field report). Found by substring on the component's class or
// object name (maghidename, default "Magazine"); magdump surveys the held weapon's components so
// the right substring can be read off rather than guessed.
struct FRawArrayRO_ { void* data; int32_t num; int32_t max; };

TrackedObject s_mag_hidden;   // the component we hid -- what release must undo

template <typename F>
void weapon_components(F&& fn) {
    auto* wpn = fp_weapon_actor();
    if (wpn == nullptr) return;
    for (const wchar_t* prop : {L"BlueprintCreatedComponents", L"InstanceComponents"}) {
        auto* arr = wpn->get_property_data<FRawArrayRO_>(prop);
        if (arr == nullptr || IsBadReadPtr(arr, sizeof(FRawArrayRO_))) continue;
        if (arr->data == nullptr || arr->num <= 0 || arr->num > 4096) continue;
        auto** elems = reinterpret_cast<API::UObject**>(arr->data);
        if (IsBadReadPtr(elems, sizeof(void*) * (size_t)arr->num)) continue;
        for (int32_t i = 0; i < arr->num; ++i) {
            auto* c = elems[i];
            if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) continue;
            if (!fn(c)) return;
        }
    }
}

// The mesh ASSET a static-mesh component renders, lowercased -- the components themselves are
// anonymous (six numbered BPC_FP_StaticMesh_C on the assault rifle, field-surveyed), and the
// magazine's identity lives entirely in the assigned SM_*_Magazine asset.
std::wstring comp_mesh_lname(API::UObject* c) {
    auto** mesh = c->get_property_data<API::UObject*>(L"StaticMesh");
    if (mesh == nullptr || IsBadReadPtr(mesh, sizeof(void*))) return L"";
    auto* m = *mesh;
    if (m == nullptr || IsBadReadPtr(m, sizeof(void*))) return L"";
    std::wstring full = m->get_full_name();
    for (auto& ch : full) ch = (wchar_t)towlower(ch);
    return full;
}

void mag_set_visible(API::UObject* comp, bool visible) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[64] = {0};
    p[0] = visible ? 1 : 0;
    p[1] = 1;   // bPropagateToChildren -- the mag may carry child meshes
    comp->call_function(L"SetVisibility", p);
}

// How the component in s_mag_hidden was hidden (reload_state_hide), so the release undoes that
// and only that; the actor it was found on (compared, never dereferenced); its own scale.
int           s_mag_hid_mode = -1;
API::UObject* s_mag_hid_actor = nullptr;
double        s_mag_hid_scale[3] = {1.0, 1.0, 1.0};
long long     s_mag_find_at = 0;
uint32_t      s_mag_rehides = 0;

void mag_set_hidden_in_game(API::UObject* comp, bool hidden) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[64] = {0};
    p[0] = hidden ? 1 : 0;
    p[1] = 1;   // bPropagateToChildren
    comp->call_function(L"SetHiddenInGame", p);
}
void mag_set_rel_scale(API::UObject* comp, const double s[3]) {
    if (comp == nullptr) return;
    alignas(16) uint8_t p[64] = {0};
    memcpy(p, s, sizeof(double) * 3);   // FVector, three doubles
    comp->call_function(L"SetRelativeScale3D", p);
}
double* mag_rel_scale(API::UObject* comp) {
    auto* s = comp->get_property_data<double>(L"RelativeScale3D");
    if (s == nullptr || IsBadReadPtr(s, sizeof(double) * 3)) return nullptr;
    return s;
}
void mag_hide_comp(API::UObject* c, int mode, bool first) {
    if (mode == 2) { mag_set_hidden_in_game(c, true); return; }
    if (mode == 3) {
        if (first) {
            if (auto* s = mag_rel_scale(c))
                for (int i = 0; i < 3; ++i) s_mag_hid_scale[i] = (s[i] > 0.01) ? s[i] : 1.0;   // never capture our own 0.001
        }
        const double z[3] = {0.001, 0.001, 0.001};
        mag_set_rel_scale(c, z);
        return;
    }
    mag_set_visible(c, false);
}
void mag_unhide_comp(API::UObject* c, int mode) {
    if (c == nullptr) return;
    if (mode == 2) { mag_set_hidden_in_game(c, false); return; }
    if (mode == 3) { mag_set_rel_scale(c, s_mag_hid_scale); return; }
    mag_set_visible(c, true);
}

void mag_hide_apply(bool out) {
    if (!g_cfg.mag_hide || g_cfg.mag_hide_name[0] == 0) return;
    if (!out) {
        // Restore whatever we hid, wherever the weapon has since gone. A dead handle just means
        // the actor was torn down, which hid it more thoroughly than we ever could.
        if (auto* c = s_mag_hidden.get()) mag_unhide_comp(c, s_mag_hid_mode);
        s_mag_hidden = TrackedObject{};
        s_mag_hid_actor = nullptr; s_mag_hid_mode = -1;
        return;
    }
    if (s_mag_hidden.get() != nullptr) return;
    std::wstring want;
    for (const char* q = g_cfg.mag_hide_name; *q != 0; ++q)
        want.push_back((wchar_t)towlower((unsigned char)*q));
    const int mode = (g_cfg.reload_state_id == 0 || g_cfg.reload_state_hide == 0) ? 1 : g_cfg.reload_state_hide;
    weapon_components([&](API::UObject* c) {
        // Match on the component's class or name, OR on the ASSET its mesh renders -- the
        // components are anonymous and the magazine's name lives in the asset.
        const auto* fn = c->get_fname();
        std::wstring nm = (fn != nullptr) ? fn->to_string() : L"";
        std::wstring cl = class_name_of(c);
        for (auto& ch : nm) ch = (wchar_t)towlower(ch);
        for (auto& ch : cl) ch = (wchar_t)towlower(ch);
        const std::wstring mesh = comp_mesh_lname(c);
        // The classic assault rifle's asset is spelled "Megazine" (pak survey); the default name
        // finds it too.
        const bool mega = (want == L"magazine") && mesh.find(L"megazine") != std::wstring::npos;
        if (cl.find(want) == std::wstring::npos && nm.find(want) == std::wstring::npos &&
            mesh.find(want) == std::wstring::npos && !mega) return true;
        mag_hide_comp(c, mode, true);
        s_mag_hidden.set(c);
        s_mag_hid_mode = mode;
        s_mag_hid_actor = fp_weapon_actor();
        return false;   // first match wins
    });
}

// THE MAGAZINE STAYS OUT ON WHATEVER RENDERS THE WEAPON NOW (reload_state_hide). The hide used to
// land once, on one component, at the state change. A weapon swap spawns new components (Arms.cpp
// and Rig.cpp both record it), and the weapon actor is re-resolved only every ~60 ticks, so a swap
// away and back inside that window presents a fresh actor under the SAME weapon key: no state
// change, so no hide, and the fresh magazine is drawn while the machine still says MAG_OUT. The
// same holds for any rebuild of the first-person weapon (a seat, a respawn of the FP build).
void mag_hide_enforce() {
    if (!g_cfg.mag_hide || g_cfg.mag_hide_name[0] == 0) return;
    if (g_cfg.reload_state_id == 0 || g_cfg.reload_state_hide == 0) return;
    if (s_reload == ReloadState::Idle) return;
    auto* actor = fp_weapon_actor();
    if (actor == nullptr) return;   // nothing renders the weapon this tick
    const int mode = g_cfg.reload_state_hide;
    auto* c = s_mag_hidden.get();
    if (c != nullptr && (s_mag_hid_actor != actor || s_mag_hid_mode != mode)) {
        if (g_cfg.reload_state_log)
            API::get()->log_info("[Halo-CampE-UEVR] RSTATE maghide: %s, the old component is released and the magazine re-found",
                                 s_mag_hid_actor != actor ? "a different actor renders the weapon" : "the hide mode changed");
        mag_unhide_comp(c, s_mag_hid_mode);
        s_mag_hidden = TrackedObject{}; s_mag_hid_actor = nullptr; s_mag_hid_mode = -1;
        c = nullptr;
    }
    if (c == nullptr) {
        const long long nowt = now_ticks();
        if (nowt - s_mag_find_at < ms_to_ticks(250)) return;
        s_mag_find_at = nowt;
        s_mag_hidden = TrackedObject{};
        mag_hide_apply(true);
        if (g_cfg.reload_state_log) {
            auto* h = s_mag_hidden.get();
            API::get()->log_info("[Halo-CampE-UEVR] RSTATE maghide: %s on %ls (mode %d, state %d)",
                                 h ? "magazine hidden" : "NO magazine component found", class_name_of(actor).c_str(), mode, (int)s_reload);
            if (h != nullptr && mode == 1) {
                static bool s_said = false;
                if (!s_said) { s_said = true; API::get()->log_info("[Halo-CampE-UEVR] RSTATE maghide: IsVisible is %s on %ls", h->get_class()->find_function(L"IsVisible") ? "reflected" : "NOT reflected (re-hide cannot detect a re-show)", class_name_of(h).c_str()); }
            }
        }
        return;
    }
    bool rehid = false;
    if (mode == 1) {
        alignas(16) uint8_t p[64] = {0};
        c->call_function(L"IsVisible", p);
        if (p[0] != 0) { mag_set_visible(c, false); rehid = true; }
    } else if (mode == 2) {
        mag_set_hidden_in_game(c, true);
    } else if (mode == 3) {
        if (auto* s = mag_rel_scale(c)) { if (s[0] > 0.01 || s[1] > 0.01 || s[2] > 0.01) { mag_hide_comp(c, 3, false); rehid = true; } }
    }
    if (rehid) {
        ++s_mag_rehides;
        if (g_cfg.reload_state_log && (s_mag_rehides <= 20u || (s_mag_rehides % 100u) == 0u))
            API::get()->log_info("[Halo-CampE-UEVR] RSTATE maghide: the magazine was showing again on %ls, re-hidden (mode %d, %u so far)",
                                 class_name_of(actor).c_str(), mode, s_mag_rehides);
    }
}

// ---- THE WELL MARKER: a small ring at the insert point while the magazine is in hand. The
// seat test has always known where the well is; the player could not see it (a tester video,
// 2026-09-01: testers had no idea where the mag goes). TrackedObject + paced respawn, the same
// lifecycle every holster marker uses -- a level load recycles the component and the next
// MagHeld tick rebuilds it.
TrackedObject s_well_marker;
long long s_well_try_at = 0;
void reload_well_marker_update(bool show, const Vec3& world) {
    if (!g_cfg.reload_well_marker) show = false;
    auto* m = s_well_marker.get();
    if (!show) { if (m != nullptr) { holster_marker_show(m, false); marker_render_drop(m); } return; }
    if (m == nullptr) {
        const long long nowt = now_ticks();
        if (nowt - s_well_try_at < ms_to_ticks(2000)) return;
        s_well_try_at = nowt;
        auto* owner = API::get()->get_local_pawn(0);
        if (owner == nullptr) return;
        static const wchar_t* kWellRing[] = {
            L"StaticMesh /Engine/BasicShapes/Torus.Torus",
            L"StaticMesh /Engine/BasicShapes/Sphere.Sphere",
        };
        const double s = (double)g_cfg.reload_well_marker_scale;
        m = marker_spawn_list(owner, kWellRing, 2, s, s, s * 0.5);
        if (m == nullptr) return;
        marker_tint(m, g_cfg.well_marker_color);
        s_well_marker.set(m);
    }
    holster_marker_place(m, world);
    holster_marker_show(m, true);
    {   // the one solve: room-anchored like everything else, re-placed per frame (Markers.hpp)
        Vec3 hp{}; Quat hr{};
        if (get_pose(API::VR::get_hmd_index(), &hp, &hr, /*use_aim=*/false))
            marker_render_anchor(m, holster_world_to_room(world, hp));
    }
}

// The slide's clock: 0 = not sliding. Reset whenever the reload leaves MagHeld by any route.
long long s_slide_start = 0;
void reload_slide_reset() {
    s_slide_start = 0;
    g_reload_slide_t.store(-1.0f, std::memory_order_relaxed);
}

// MAGDUMP: the held weapon's component roster, one-shot on value change.
void mag_dump_probe() {
    static int s_armed_as = 0;
    if (g_cfg.mag_dump == s_armed_as) return;
    s_armed_as = g_cfg.mag_dump;
    if (s_armed_as == 0) return;
    auto* wpn = fp_weapon_actor();
    API::get()->log_info("[Halo-CampE-UEVR] MAGDUMP %d: weapon %ls",
                         s_armed_as, wpn ? class_name_of(wpn).c_str() : L"(none)");
    int n = 0;
    weapon_components([&](API::UObject* c) {
        const auto* fn = c->get_fname();
        const std::wstring mesh = comp_mesh_lname(c);
        API::get()->log_info("[Halo-CampE-UEVR] MAGDUMP   %ls '%ls'%s%ls",
                             class_name_of(c).c_str(),
                             (fn != nullptr) ? fn->to_string().c_str() : L"?",
                             mesh.empty() ? "" : "  mesh=",
                             mesh.c_str());
        return ++n < 64;
    });
    API::get()->log_info("[Halo-CampE-UEVR] MAGDUMP %d done: %d components", s_armed_as, n);
}

// ANIMDUMP: where does the reload animation come from? The palette hold wrote 89 frames over
// both render banks and the magazine still cycled (2026-09-02), so that motion is NOT in the
// Blam palette we own. For ~2.5 s after a seat, sample every skeletal component on the weapon
// and the arms rig: animation mode, anim-instance class, the active montage, IsPlaying. A
// montage name appearing here IS the answer, and the fix follows from it.
long long s_anim_dump_until = 0;
void anim_dump_tick() {
    if (s_anim_dump_until == 0) return;
    const long long nowt = now_ticks();
    if (nowt > s_anim_dump_until) {
        s_anim_dump_until = 0;
        API::get()->log_info("[Halo-CampE-UEVR] ANIMDUMP end");
        return;
    }
    static long long s_last = 0;
    if (nowt - s_last < ms_to_ticks(120)) return;
    s_last = nowt;
    auto report = [&](API::UObject* c, const char* who) {
        if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) return;
        const std::wstring cl = class_name_of(c);
        // Audio components too -- the reload SOUND's home is the next question, and whether the
        // weapon actor owns an AudioComponent decides whether a local mute is even on the table.
        if (cl.find(L"Audio") != std::wstring::npos) {
            const auto* fn = c->get_fname();
            API::get()->log_info("[Halo-CampE-UEVR] ANIMDUMP %s AUDIO %ls '%ls'", who, cl.c_str(),
                                 fn ? fn->to_string().c_str() : L"?");
            return;
        }
        if (cl.find(L"Skeletal") == std::wstring::npos && cl.find(L"Skinned") == std::wstring::npos) return;
        int mode = -1;
        if (auto* pm = c->get_property_data<uint8_t>(L"AnimationMode"))
            if (!IsBadReadPtr(pm, 1)) mode = (int)*pm;
        API::UObject* animbp = nullptr;
        if (auto** pai = c->get_property_data<API::UObject*>(L"AnimScriptInstance"))
            if (!IsBadReadPtr(pai, sizeof(void*))) animbp = *pai;
        std::wstring aicl = L"-", mont = L"-";
        if (animbp != nullptr && !IsBadReadPtr(animbp, sizeof(void*))) {
            aicl = class_name_of(animbp);
            alignas(16) uint8_t p[64] = {0};
            animbp->call_function(L"GetCurrentActiveMontage", p);
            auto* m = *reinterpret_cast<API::UObject**>(p);
            if (m != nullptr && !IsBadReadPtr(m, sizeof(void*))) mont = m->get_full_name();
        }
        bool playing = false;
        { alignas(16) uint8_t p[64] = {0}; c->call_function(L"IsPlaying", p); playing = p[0] != 0; }
        float rate = -1.0f;
        if (auto* pr = c->get_property_data<float>(L"GlobalAnimRateScale"))
            if (!IsBadReadPtr(pr, sizeof(float))) rate = *pr;
        const auto* fn = c->get_fname();
        API::get()->log_info("[Halo-CampE-UEVR] ANIMDUMP %s %ls '%ls' mode=%d anim=%ls montage=%ls playing=%d rate=%.1f",
                             who, cl.c_str(), fn ? fn->to_string().c_str() : L"?",
                             mode, aicl.c_str(), mont.c_str(), (int)playing, rate);
    };
    weapon_components([&](API::UObject* c) { report(c, "weapon"); return true; });
    report(rig_tracked_component(), "arms");
}

// ---- FAST-FORWARD THE GAME'S RELOAD ANIMATION. ANIMDUMP (2026-09-02) settled where it lives:
// an Animation Blueprint on the weapon's skeletal mesh (ABP_FP_<Weapon>_Default_C) and one on
// the arms, no montage, nothing in the Blam palette. The player has just seated the magazine
// by hand; the arms then acting it out again is the wrong film, and freezing the pose was
// refused ("I don't want anything frozen"). So the animation is not stopped -- it is run at
// reload_anim_rate x for reload_anim_ms after the seat: GlobalAnimRateScale on every skeletal
// component of the weapon and the arms, restored to 1.0 when the window closes. The reload
// still happens, the ammo still lands, the animation is a blink. Components are remembered so
// the restore hits exactly what was touched, and a dead handle simply means the actor is gone.
TrackedObject s_anim_rate_comps[6];
int           s_anim_rate_n = 0;
long long     s_anim_rate_until = 0;
void reload_anim_rate_set(API::UObject* c, float rate) {
    if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) return;
    auto* p = c->get_property_data<float>(L"GlobalAnimRateScale");
    if (p == nullptr || IsBadWritePtr(p, sizeof(float))) return;
    *p = rate;
}
// A bool UPROPERTY written through its bitfield mask -- bPauseAnims shares a byte with its
// neighbours, and a raw byte write would flip them too (the read side of this lesson is
// read_bool_prop in Plugin.cpp). Returns false when the class has no such property.
bool reload_set_bool_prop(API::UObject* obj, const wchar_t* name, bool value) {
    if (obj == nullptr || IsBadReadPtr(obj, sizeof(void*))) return false;
    auto* cls = obj->get_class();
    if (cls == nullptr) return false;
    auto* prop = cls->find_property(name);
    if (prop == nullptr) return false;
    static_cast<API::FBoolProperty*>(prop)->set_value_in_object(obj, value);
    return true;
}
// PAUSE THE ANIMATION UPDATE for the window (reloadpauseanim). The state clamp alone loses one
// frame every time: the game writes the reload state during its actor tick, after ours, and
// the AnimBP evaluates that frame before the next clamp -- measured "re-asserted 1 times" on
// every reload after a restart, and that single frame is what fires the Blam-side reload sound.
// bPauseAnims skips the animation update entirely, so the transition is never evaluated; by the
// time the update resumes the clamp has the variable back at idle. The gun still rides the hand
// (that is the component transform, not the animation).
void reload_anim_rate_begin() {
    const bool want_rate  = g_cfg.reload_anim_rate > 0.0f;
    const bool want_pause = g_cfg.reload_pause_anim;
    if ((!want_rate && !want_pause) || g_cfg.reload_anim_ms <= 0) return;
    s_anim_rate_n = 0;
    int paused = 0;
    auto grab = [&](API::UObject* c) {
        if (c == nullptr || s_anim_rate_n >= 6) return;
        const std::wstring cl = class_name_of(c);
        if (cl.find(L"Skeletal") == std::wstring::npos && cl.find(L"Skinned") == std::wstring::npos) return;
        if (want_rate)  reload_anim_rate_set(c, g_cfg.reload_anim_rate);
        if (want_pause && reload_set_bool_prop(c, L"bPauseAnims", true)) ++paused;
        s_anim_rate_comps[s_anim_rate_n++].set(c);
    };
    weapon_components([&](API::UObject* c) { grab(c); return s_anim_rate_n < 6; });
    grab(rig_tracked_component());
    s_anim_rate_until = now_ticks() + ms_to_ticks(g_cfg.reload_anim_ms);
    if (g_cfg.reload_vr_log)
        API::get()->log_info("[Halo-CampE-UEVR] RELOAD anim window: %d components, rate x%.0f%s, paused %d, for %d ms",
                             s_anim_rate_n, want_rate ? g_cfg.reload_anim_rate : 1.0f,
                             want_rate ? "" : " (off)", paused, g_cfg.reload_anim_ms);
}
void reload_anim_rate_tick() {
    if (s_anim_rate_until == 0) return;
    if (now_ticks() < s_anim_rate_until) return;
    for (int i = 0; i < s_anim_rate_n; ++i) {
        auto* c = s_anim_rate_comps[i].get();
        reload_anim_rate_set(c, 1.0f);
        reload_set_bool_prop(c, L"bPauseAnims", false);
        s_anim_rate_comps[i] = TrackedObject{};
    }
    s_anim_rate_n = 0;
    s_anim_rate_until = 0;
}

// ---- ANIMVARS: which Animation Blueprint variable IS the reload? The AnimBPs (ANIMDUMP) hold
// the reload as a state in their graph, driven by variables on the anim instance. Snapshot every
// scalar property of the weapon's and arms' anim instances at the seat, re-read at +350 ms and
// +900 ms, log only what changed. Holding the one that flips is "the animation does not run".
struct AnimVar { std::wstring name; std::wstring cls; int32_t off; int kind; double v; };
std::vector<AnimVar> s_av_base[2];
TrackedObject        s_av_obj[2];
const char*          s_av_who[2] = {"weapon", "arms"};
long long            s_av_at = 0;
int                  s_av_phase = 0;

int av_kind(const std::wstring& c) {
    if (c == L"BoolProperty")   return 1;
    if (c == L"ByteProperty" || c == L"EnumProperty") return 2;
    if (c == L"IntProperty")    return 3;
    if (c == L"FloatProperty")  return 4;
    if (c == L"DoubleProperty") return 5;
    return 0;
}
double av_read(API::UObject* o, int32_t off, int kind) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(o) + off;
    if (off < 0 || off > 0x4000 || IsBadReadPtr(p, 8)) return -999999.0;
    switch (kind) {
        case 1: case 2: return (double)*p;
        case 3: return (double)*reinterpret_cast<const int32_t*>(p);
        case 4: return (double)*reinterpret_cast<const float*>(p);
        case 5: return *reinterpret_cast<const double*>(p);
    }
    return 0.0;
}
void av_collect(API::UObject* animbp, std::vector<AnimVar>& out) {
    out.clear();
    if (animbp == nullptr || IsBadReadPtr(animbp, sizeof(void*))) return;
    for (API::UStruct* s = animbp->get_class(); s != nullptr && out.size() < 600; s = s->get_super_struct()) {
        if (IsBadReadPtr(s, sizeof(void*))) break;
        for (API::FField* f = s->get_child_properties(); f != nullptr; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            auto* fc = f->get_class();
            if (fc == nullptr || IsBadReadPtr(fc, sizeof(void*))) continue;
            const std::wstring cls = fc->get_name();
            const int kind = av_kind(cls);
            if (kind == 0) continue;
            auto* fp = static_cast<API::FProperty*>(f);
            const int32_t off = fp->get_offset();
            const auto* fn = f->get_fname();
            out.push_back(AnimVar{fn ? fn->to_string() : L"?", cls, off, kind, av_read(animbp, off, kind)});
            if (out.size() >= 600) break;
        }
    }
}
API::UObject* av_instance_of(API::UObject* comp) {
    if (comp == nullptr || IsBadReadPtr(comp, sizeof(void*))) return nullptr;
    auto** pai = comp->get_property_data<API::UObject*>(L"AnimScriptInstance");
    if (pai == nullptr || IsBadReadPtr(pai, sizeof(void*))) return nullptr;
    return *pai;
}
// ANIMOBJS: the OBJECT references on an object -- every ObjectProperty / WeakObjectProperty with
// the class and name of what it points at. The scalar roster cannot show where an AnimBP copies
// its pose FROM (bUsingCopyPoseFromMesh=1 on both), and that source mesh is the trail to the
// weapon's own Blam palette, where the slide bone really lives (2026-09-03).
void anim_objs_dump(API::UObject* o, const char* who) {
    if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) return;
    int n = 0;
    for (API::UStruct* s = o->get_class(); s != nullptr && n < 120; s = s->get_super_struct()) {
        if (IsBadReadPtr(s, sizeof(void*))) break;
        const std::wstring sn = s->get_fname() ? s->get_fname()->to_string() : L"?";
        for (API::FField* f = s->get_child_properties(); f != nullptr && n < 120; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            auto* fc = f->get_class();
            if (fc == nullptr) continue;
            const std::wstring cls = fc->get_name();
            if (cls != L"ObjectProperty" && cls != L"WeakObjectProperty" && cls != L"SoftObjectProperty") continue;
            auto* fp = static_cast<API::FProperty*>(f);
            const int32_t off = fp->get_offset();
            if (off < 0 || off > 0x8000) continue;
            auto** pp = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(o) + off);
            if (IsBadReadPtr(pp, sizeof(void*))) continue;
            API::UObject* v = *pp;
            std::wstring vdesc = L"null";
            if (v != nullptr && !IsBadReadPtr(v, sizeof(void*))) {
                const auto* vn = v->get_fname();
                vdesc = class_name_of(v) + L" '" + (vn ? vn->to_string() : L"?") + L"'";
            }
            const auto* fn = f->get_fname();
            API::get()->log_info("[Halo-CampE-UEVR] ANIMOBJS %s %ls::%ls (%ls@0x%X) -> %ls",
                                 who, sn.c_str(), fn ? fn->to_string().c_str() : L"?", cls.c_str(),
                                 (unsigned)off, vdesc.c_str());
            ++n;
        }
        if (sn == L"Object") break;
    }
}

void audio_dump_fields(API::UObject* c);   // defined below; the bridge dump borrows its lister
void anim_vars_begin() {
    API::UObject* wcomp = nullptr;
    weapon_components([&](API::UObject* c) {
        const auto* fn = c->get_fname();
        if (class_name_of(c) == L"SkeletalMeshComponent" && fn != nullptr && fn->to_string() == L"Default") {
            wcomp = c; return false;
        }
        return true;
    });
    API::UObject* inst[2] = { av_instance_of(wcomp), av_instance_of(rig_tracked_component()) };
    if (g_cfg.anim_objs) {
        anim_objs_dump(wcomp, "weapon-comp");
        anim_objs_dump(inst[0], "weapon-anim");
        anim_objs_dump(fp_weapon_actor(), "weapon-actor");
        // THE BRIDGE. BlamMeshSynchronizationComponent (found 2026-09-03) is what carries Blam's
        // weapon-object node matrices into the UE mesh -- the slide's continuous pose during the
        // game's own animations comes through it. Its fields say what it syncs FROM.
        if (auto* wpn = fp_weapon_actor()) {
            for (const wchar_t* prop : {L"BlamMeshSynchronization", L"BlamObjectSynchronization", L"BlamWeapon"}) {
                auto** pc = wpn->get_property_data<API::UObject*>(prop);
                if (pc == nullptr || IsBadReadPtr(pc, sizeof(void*)) || *pc == nullptr) continue;
                API::get()->log_info("[Halo-CampE-UEVR] ANIMOBJS ---- %ls (%ls)", prop, class_name_of(*pc).c_str());
                anim_objs_dump(*pc, "bridge");
                audio_dump_fields(*pc);
                // VALUES of the scalar fields too -- BlamObjectIndex is the Blam object datum
                // this actor mirrors, the handle to the weapon object's own node matrices.
                std::vector<AnimVar> vals;
                av_collect(*pc, vals);
                for (const auto& v : vals)
                    if (v.cls == L"IntProperty" || v.cls == L"FloatProperty")
                        API::get()->log_info("[Halo-CampE-UEVR] ANIMOBJS %ls value %ls = %.0f (0x%X)",
                                             prop, v.name.c_str(), v.v, (unsigned)(int32_t)v.v);
            }
        }
    }
    for (int i = 0; i < 2; ++i) {
        s_av_obj[i] = TrackedObject{};
        if (inst[i] != nullptr) { s_av_obj[i].set(inst[i]); av_collect(inst[i], s_av_base[i]); }
        API::get()->log_info("[Halo-CampE-UEVR] ANIMVARS %s: %s, %zu scalar vars snapshotted at the seat",
                             s_av_who[i], inst[i] ? narrow(class_name_of(inst[i])).c_str() : "(no instance)",
                             s_av_base[i].size());
        // The full roster, not just what changed: a slide/bolt/chamber variable that the reload
        // does not touch is exactly what a hand-racked slide would drive (2026-09-03).
        for (const auto& v : s_av_base[i])
            API::get()->log_info("[Halo-CampE-UEVR] ANIMVARS %s   %ls (%ls@0x%X) = %.3f",
                                 s_av_who[i], v.name.c_str(), v.cls.c_str(), (unsigned)v.off, v.v);
    }
    s_av_at = now_ticks();
    s_av_phase = 1;
}
void anim_vars_tick() {
    if (s_av_phase == 0) return;
    const long long since = now_ticks() - s_av_at;
    const int want_ms = (s_av_phase == 1) ? 350 : 900;
    if (since < ms_to_ticks(want_ms)) return;
    for (int i = 0; i < 2; ++i) {
        auto* animbp = s_av_obj[i].get();
        if (animbp == nullptr) continue;
        std::vector<AnimVar> now;
        av_collect(animbp, now);
        int changed = 0;
        for (size_t k = 0; k < now.size() && k < s_av_base[i].size(); ++k) {
            if (now[k].off != s_av_base[i][k].off) continue;
            if (now[k].v == s_av_base[i][k].v) continue;
            ++changed;
            if (changed <= 40)
                API::get()->log_info("[Halo-CampE-UEVR] ANIMVARS %s +%dms: %ls (%ls@0x%X) %.3f -> %.3f",
                                     s_av_who[i], want_ms, now[k].name.c_str(), now[k].cls.c_str(),
                                     (unsigned)now[k].off, s_av_base[i][k].v, now[k].v);
        }
        API::get()->log_info("[Halo-CampE-UEVR] ANIMVARS %s +%dms: %d of %zu changed", s_av_who[i], want_ms, changed, now.size());
    }
    s_av_phase = (s_av_phase == 1) ? 2 : 0;
}

// ---- HOLD THE RELOAD STATE. ANIMVARS (2026-09-02, assault rifle): the weapon's AnimBP carries
// FirstPersonState (enum, 0 idle -> 6 for the reload) and flips AnimToggle to retrigger; the
// arms' instance changes nothing, it follows the weapon. Holding FirstPersonState at the idle
// value for the window is "the animation does not run" -- IF the game does not re-assert the
// variable after us each frame (our tick runs before the actors'). Not assumed: the value is
// read BEFORE every write and a non-idle read is counted as a re-assertion, and the summary
// line at the window's end says how many times the game fought back. Zero means the hold wins.
TrackedObject s_sh_inst;
long long     s_sh_until = 0;
long long     s_sh_press_at = 0;      // when this hold began
long long     g_last_refill_at = 0;   // the rounds counter last jumped up (slide_phantom_tick)
bool net_is_coop();
uint8_t       s_sh_idle = 0;
int           s_sh_reasserts = 0, s_sh_ticks = 0;
API::UObject* reload_weapon_default_comp() {
    // Memoised per weapon actor for 8 ms: the tick asks for this component eight or more times,
    // and each walk built two strings per component (perf audit, 2026-09-06). The actor itself is
    // validated by fp_weapon_actor(), so a component of a live actor is live.
    static API::UObject* s_memo_actor = nullptr; static API::UObject* s_memo = nullptr; static long long s_memo_at = 0;
    auto* actor = fp_weapon_actor();
    const long long nowt = now_ticks();
    if (actor != nullptr && actor == s_memo_actor && s_memo != nullptr && nowt - s_memo_at < ms_to_ticks(8)) return s_memo;
    API::UObject* wcomp = nullptr;
    weapon_components([&](API::UObject* c) {
        const auto* fn = c->get_fname();
        if (class_name_of(c) == L"SkeletalMeshComponent" && fn != nullptr && fn->to_string() == L"Default") {
            wcomp = c; return false;
        }
        return true;
    });
    s_memo_actor = actor; s_memo = wcomp; s_memo_at = nowt;
    return wcomp;
}
API::UObject* reload_weapon_anim_instance() {
    return av_instance_of(reload_weapon_default_comp());
}

// ---- ANIMSEQSET (dev): pose the weapon mesh from a named AnimSequence at a fixed time --
// "animseqset=Magnum_first_person_fire,0.05". The utoc (2026-09-03) shows the first-person
// weapon animations are UE AnimSequence assets (A_Magnum_first_person_fire_1_var1, the reloads,
// the two-state ammunition pose), so the slide is a UE bone those sequences pose. Single-node
// mode on the FIRE sequence, time scrubbed by the hand, IS smooth slide travel; this key finds
// the time where the slide sits fully back. Clearing the key hands the mesh back to its AnimBP.
TrackedObject s_as_seq;
std::string   s_as_last;
bool          s_as_active = false;
API::UObject* find_anim_sequence(const std::string& sub) {
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return nullptr;
    std::wstring want(sub.begin(), sub.end());
    for (auto& ch : want) ch = (wchar_t)towlower(ch);
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AnimSequence") continue;
        const auto* fn = o->get_fname();
        if (fn == nullptr) continue;
        std::wstring nm = fn->to_string();
        for (auto& ch : nm) ch = (wchar_t)towlower(ch);
        if (nm.find(want) != std::wstring::npos) return o;
    }
    return nullptr;
}
void anim_seq_release(API::UObject* comp) {
    if (comp != nullptr && !IsBadReadPtr(comp, sizeof(void*))) {
        alignas(16) uint8_t p[64] = {0}; p[0] = 0;   // EAnimationMode::AnimationBlueprint
        comp->call_function(L"SetAnimationMode", p);
    }
    s_as_active = false; s_as_last.clear(); s_as_seq = TrackedObject{};
}
void anim_seq_set_tick() {
    const std::string spec = g_cfg.anim_seq_set;
    auto* comp = reload_weapon_default_comp();
    if (spec.empty()) { if (s_as_active) { anim_seq_release(comp); API::get()->log_info("[Halo-CampE-UEVR] ANIMSEQSET: released to AnimBP"); } return; }
    if (comp == nullptr) return;
    const size_t comma = spec.find(',');
    if (comma == std::string::npos) return;
    const std::string name(spec.substr(0, comma));
    const float time = (float)atof(spec.c_str() + comma + 1);
    if (spec != s_as_last) {
        s_as_last = spec;
        auto* seq = find_anim_sequence(name);
        if (seq == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] ANIMSEQSET: no AnimSequence matching '%s'", name.c_str()); return; }
        s_as_seq.set(seq);
        { alignas(16) uint8_t p[64] = {0}; p[0] = 1;   // EAnimationMode::AnimationSingleNode
          comp->call_function(L"SetAnimationMode", p); }
        { alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<API::UObject**>(p) = seq;
          comp->call_function(L"SetAnimation", p); }
        s_as_active = true;
        API::get()->log_info("[Halo-CampE-UEVR] ANIMSEQSET: %ls on %ls, single-node, time=%.3f",
                             seq->get_full_name().c_str(), class_name_of(comp).c_str(), time);
    }
    if (s_as_seq.get() == nullptr) return;
    alignas(16) uint8_t p[64] = {0};
    *reinterpret_cast<float*>(p) = time;
    p[4] = 0;   // bFireNotifies = false
    comp->call_function(L"SetPosition", p);
}
void reload_state_hold_begin() {
    if (!g_cfg.reload_hold_state || g_cfg.reload_anim_ms <= 0) return;
    auto* animbp = reload_weapon_anim_instance();
    if (animbp == nullptr) return;
    auto* p = animbp->get_property_data<uint8_t>(L"FirstPersonState");
    if (p == nullptr || IsBadReadPtr(p, 1)) {
        if (g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] STATEHOLD: no FirstPersonState on %ls", class_name_of(animbp).c_str());
        return;
    }
    s_sh_idle = *p;
    s_sh_inst.set(animbp);
    s_sh_press_at = now_ticks();
    s_sh_until = s_sh_press_at + ms_to_ticks((g_cfg.coop_auto && net_is_coop()) ? g_cfg.reload_anim_ms_coop : g_cfg.reload_anim_ms);
    s_sh_reasserts = 0; s_sh_ticks = 0;
}
bool reload_gestures_busy();   // the mag is out, the seat is pending, or the lock waits for the rack (defined below)
void reload_state_hold_tick() {
    if (s_sh_until == 0) return;
    auto* animbp = s_sh_inst.get();
    // The hold outlives reload_anim_ms while OUR reload is still in progress (the shotgun's
    // shell-by-shell animation ran on past 1.2 s and showed, 2026-09-06), capped at +6 s.
    const long long nowt = now_ticks();
    // Coop: the refill is the proof the game's reload ran; until it lands the hold stays (capped).
    const bool coop_wait = g_cfg.coop_auto && net_is_coop() && g_last_refill_at < s_sh_press_at && nowt < s_sh_until + ms_to_ticks(4000);
    const bool over = nowt >= s_sh_until && !coop_wait && (!reload_gestures_busy() || nowt >= s_sh_until + ms_to_ticks(6000));
    if (animbp == nullptr || over) {
        if (g_cfg.reload_vr_log)
            API::get()->log_info("[Halo-CampE-UEVR] STATEHOLD: held FirstPersonState=%d for %d ticks, game re-asserted %d times%s",
                                 (int)s_sh_idle, s_sh_ticks, s_sh_reasserts,
                                 animbp == nullptr ? " (instance gone)" : "");
        s_sh_until = 0; s_sh_inst = TrackedObject{};
        return;
    }
    auto* p = animbp->get_property_data<uint8_t>(L"FirstPersonState");
    if (p == nullptr || IsBadWritePtr(p, 1)) return;
    ++s_sh_ticks;
    if (*p != s_sh_idle) { ++s_sh_reasserts; *p = s_sh_idle; }
}

// ---- AUDIODUMP: where does the reload SOUND come from? The state hold killed the animation
// and the sound still played (2026-09-02), so it is not an anim notify; the weapon carries a
// HaloAudioTrackingComponent, which suggests the Blam sim's sound event lands in UE audio via
// that. Two sweeps after the seat (+120 ms, +400 ms) over the object array for AudioComponents
// that are PLAYING -- owner, sound asset -- plus a one-shot field listing of the tracking
// component. Whatever is playing on the weapon during the window is the mute target.
long long s_ad_at = 0;
int       s_ad_phase = 0;
void audio_dump_fields(API::UObject* c) {
    if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) return;
    int n = 0;
    for (API::UStruct* s = c->get_class(); s != nullptr && n < 80; s = s->get_super_struct()) {
        if (IsBadReadPtr(s, sizeof(void*))) break;
        const std::wstring sn = s->get_fname() ? s->get_fname()->to_string() : L"?";
        if (sn.find(L"Actor") != std::wstring::npos && sn.find(L"Component") == std::wstring::npos) break;
        for (API::FField* f = s->get_child_properties(); f != nullptr && n < 80; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            auto* fc = f->get_class();
            const auto* fn = f->get_fname();
            API::get()->log_info("[Halo-CampE-UEVR] AUDIODUMP field %ls::%ls (%ls @0x%X)",
                                 sn.c_str(), fn ? fn->to_string().c_str() : L"?",
                                 fc ? fc->get_name().c_str() : L"?",
                                 (unsigned)static_cast<API::FProperty*>(f)->get_offset());
            ++n;
        }
        if (sn == L"SceneComponent" || sn == L"ActorComponent") break;
    }
}
std::string trim_cfg(const char* v);
extern bool s_sl_empty_at_drop;
bool net_is_coop();
// ---- WWISE (see Config.hpp reload_wwise_dump). ------------------------------------------------
// THE WEAPON STEM from a skeletal mesh name (2026-09-06): "SK_" stripped, the name cut at the
// first variant suffix (_Default, _Translucent, _Mech, _Shadow, _FP), and any underscore left
// INSIDE the name removed -- the rocket launcher's mesh is SK_Rocket_Launcher, and cutting at the
// first underscore made it "Rocket", which no table matched, so its hinge never spawned.
std::wstring weapon_stem_from_mesh(std::wstring stem) {
    if (stem.rfind(L"SK_", 0) == 0) stem = stem.substr(3);
    size_t cut = std::wstring::npos;
    for (const wchar_t* suf : { L"_Default", L"_Translucent", L"_Mech", L"_Shadow", L"_FP" }) {
        const size_t at = stem.find(suf);
        if (at != std::wstring::npos && at < cut) cut = at;
    }
    if (cut != std::wstring::npos) stem = stem.substr(0, cut);
    std::wstring out; for (wchar_t ch : stem) if (ch != L'_') out += ch;
    return out;
}
std::wstring ak_weapon_stem() {
    std::wstring stem;
    auto* src = reload_weapon_default_comp();
    if (src == nullptr) return stem;
    API::UObject* mesh = nullptr;
    for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
        auto** pm = src->get_property_data<API::UObject*>(nm);
        if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
    }
    if (mesh == nullptr || IsBadReadPtr(mesh, sizeof(void*)) || mesh->get_fname() == nullptr) return stem;
    stem = mesh->get_fname()->to_string();
    if (stem.rfind(L"SK_", 0) == 0) stem = stem.substr(3);
    stem = weapon_stem_from_mesh(stem);
    for (auto& ch : stem) ch = (wchar_t)towlower(ch);
    return stem;
}
void ak_dump_function(const wchar_t* path) {
    auto* fn = API::get()->find_uobject<API::UFunction>(path);
    if (fn == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] WWISE fn %ls: NOT FOUND", path); return; }
    std::wstring sig;
    for (API::FField* f = fn->get_child_properties(); f != nullptr; f = f->get_next()) {
        if (IsBadReadPtr(f, sizeof(void*))) break;
        const auto* nm = f->get_fname(); auto* fc = f->get_class();
        if (nm == nullptr || fc == nullptr) continue;
        wchar_t buf[160]; swprintf_s(buf, L" %ls:%ls@0x%X", nm->to_string().c_str(), fc->get_name().c_str(), (unsigned)static_cast<API::FProperty*>(f)->get_offset());
        sig += buf;
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE fn %ls (params 0x%X):%ls", path, (unsigned)fn->get_properties_size(), sig.c_str());
}
void ak_dump(const char* when) {
    if (!g_cfg.reload_wwise_dump) return;
    static int s_dumps = 0;
    if (s_dumps >= 2) return;
    ++s_dumps;
    static bool s_sigs = false;
    if (!s_sigs) {
        s_sigs = true;
        ak_dump_function(L"Function /Script/AkAudio.AkGameplayStatics.PostEvent");
        ak_dump_function(L"Function /Script/AkAudio.AkGameplayStatics.PostEventByName");
        ak_dump_function(L"Function /Script/AkAudio.AkComponent.PostAkEvent");
        ak_dump_function(L"Function /Script/AkAudio.AkComponent.SetOutputBusVolume");
        ak_dump_function(L"Function /Script/AkAudio.AkComponent.Stop");
        ak_dump_function(L"Function /Script/AkAudio.AkGameplayStatics.StopActor");
    }
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    auto* wpn = fp_weapon_actor();
    const std::wstring stem = ak_weapon_stem();
    const wchar_t* keys[] = {L"reload", L"mag", L"clip", L"chamber", L"bolt", L"slide", L"rack", L"cock", L"pump", L"charg"};
    int n_ak = 0, n_ev = 0, n_comp = 0, shown = 0;
    std::wstring classes;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring cl = class_name_of(o);
        if (cl.rfind(L"Ak", 0) != 0) continue;
        ++n_ak;
        if (classes.find(L" " + cl + L" ") == std::wstring::npos && classes.size() < 900) classes += L" " + cl + L" ";
        const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
        std::wstring nm = fnm->to_string(); std::wstring lo = nm; for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        if (cl == L"AkAudioEvent") {
            ++n_ev;
            bool hit = (!stem.empty() && lo.find(stem) != std::wstring::npos);
            for (const wchar_t* k : keys) if (lo.find(k) != std::wstring::npos) { hit = true; break; }
            if (hit && shown < 160) { ++shown; API::get()->log_info("[Halo-CampE-UEVR] WWISE event %ls", nm.c_str()); }
        } else if (cl.find(L"Component") != std::wstring::npos) {
            ++n_comp;
            API::UObject* owner = nullptr;
            { alignas(16) uint8_t p[64] = {0}; o->call_function(L"GetOwner", p); owner = *reinterpret_cast<API::UObject**>(p); }
            const bool on_wpn = (owner != nullptr && wpn != nullptr && owner == wpn);
            if (shown < 200) { ++shown; API::get()->log_info("[Halo-CampE-UEVR] WWISE comp %ls '%ls' owner=%ls%s", cl.c_str(), nm.c_str(), owner ? class_name_of(owner).c_str() : L"(none)", on_wpn ? " [FP WEAPON]" : ""); }
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE dump at %s: %d Ak objects, %d events, %d components, stem '%ls'; classes:%ls", when, n_ak, n_ev, n_comp, stem.c_str(), classes.c_str());
}
// The AkAudioEvent's Wwise id property (ShortId in the 2022+ integration; a few names tried).
// The 2023 integration keeps the id in FWwiseEventCookedData (a StructProperty, no reflected
// field for the id itself). GetWwiseShortId() is reflected: the uint32 inside the cooked data
// that equals it is the id, found once per launch and the offset reused.
uint32_t ak_event_short_id(API::UObject* ev) {
    if (ev == nullptr || IsBadReadPtr(ev, sizeof(void*))) return 0;
    alignas(16) uint8_t p[64] = {0};
    ev->call_function(L"GetWwiseShortId", p);
    return *reinterpret_cast<uint32_t*>(p);
}
uint32_t* ak_event_id_ptr(API::UObject* ev) {
    static int s_off = -1;   // offset within EventCookedData
    if (ev == nullptr || IsBadReadPtr(ev, sizeof(void*))) return nullptr;
    auto* cd = ev->get_property_data<uint8_t>(L"EventCookedData");
    if (cd == nullptr || IsBadWritePtr(cd, 0x60)) return nullptr;
    if (s_off >= 0) return reinterpret_cast<uint32_t*>(cd + s_off);
    const uint32_t id = ak_event_short_id(ev);
    if (id == 0) return nullptr;
    for (int o = 0; o + 4 <= 0x60; o += 4) {
        if (*reinterpret_cast<uint32_t*>(cd + o) == id) {
            s_off = o;
            API::get()->log_info("[Halo-CampE-UEVR] WWISE: event id lives at EventCookedData+0x%X (id %u)", (unsigned)o, id);
            return reinterpret_cast<uint32_t*>(cd + o);
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE: id %u not found inside EventCookedData", id);
    return nullptr;
}
// ExecuteAction(Stop) on an event: the 2023 integration's reflected signature, placed by name.
void ak_event_stop(API::UObject* ev, API::UObject* actor) {
    if (ev == nullptr || IsBadReadPtr(ev, sizeof(void*))) return;
    auto* fn = ev->get_class() ? ev->get_class()->find_function(L"ExecuteAction") : nullptr;
    if (fn == nullptr) return;
    alignas(16) uint8_t p[128] = {0};
    if ((size_t)fn->get_properties_size() > sizeof(p)) return;
    auto put = [&](const wchar_t* name, const void* v, size_t n) { auto* pr = fn->find_property(name); if (pr == nullptr) return; const int32_t off = pr->get_offset(); if (off >= 0 && (size_t)off + n <= sizeof(p)) memcpy(p + off, v, n); };
    const uint8_t stop = 0;   // AkActionOnEventType::Stop = 0
    const int32_t dur = 0;
    put(L"ActionType", &stop, 1);
    put(L"Actor", &actor, sizeof(void*));
    put(L"TransitionDuration", &dur, sizeof(int32_t));
    ev->call_function(L"ExecuteAction", p);
}
void ak_roster(const wchar_t* cls_path) {
    auto* cls = API::get()->find_uobject<API::UClass>(cls_path);
    if (cls == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] WWISE roster %ls: NOT FOUND", cls_path); return; }
    std::wstring props, funcs;
    int np = 0;
    for (API::UStruct* st = cls; st != nullptr && np < 200; st = st->get_super_struct()) {
        if (IsBadReadPtr(st, sizeof(void*))) break;
        for (API::FField* f = st->get_child_properties(); f != nullptr && np < 200; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            const auto* nm = f->get_fname(); auto* fc = f->get_class();
            if (nm == nullptr || fc == nullptr) continue;
            wchar_t buf[160]; swprintf_s(buf, L" %ls:%ls@0x%X", nm->to_string().c_str(), fc->get_name().c_str(), (unsigned)static_cast<API::FProperty*>(f)->get_offset());
            props += buf; ++np;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE roster %ls props:%ls", cls_path, props.c_str());
    // functions: the class's children chain
    int nf = 0;
    for (API::UStruct* st = cls; st != nullptr && nf < 200; st = st->get_super_struct()) {
        if (IsBadReadPtr(st, sizeof(void*))) break;
        for (auto* c = st->get_children(); c != nullptr && nf < 200; c = c->get_next()) {
            if (IsBadReadPtr(c, sizeof(void*))) break;
            const auto* nm = c->get_fname(); if (nm == nullptr) continue;
            funcs += L" " + nm->to_string(); ++nf;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE roster %ls funcs:%ls", cls_path, funcs.c_str());
}
long long s_akd_at = 0; int s_akd_count = 0;
void ak_dump_after_press() {
    if (!g_cfg.reload_wwise_dump || s_akd_count >= 4) return;
    s_akd_at = now_ticks() + ms_to_ticks(900);
}
void ak_dump_after_press_tick() {
    if (s_akd_at == 0 || now_ticks() < s_akd_at) return;
    s_akd_at = 0; ++s_akd_count;
    static bool s_rosters = false;
    if (!s_rosters) { s_rosters = true; ak_dump_function(L"Function /Script/AkAudio.AkAudioEvent.PostOnActor"); ak_dump_function(L"Function /Script/AkAudio.AkAudioEvent.ExecuteAction"); ak_dump_function(L"Function /Script/AkAudio.AkAudioEvent.GetWwiseShortId"); ak_dump_function(L"Function /Script/AkAudio.AkAudioEvent.UnloadData"); ak_dump_function(L"Function /Script/AkAudio.AkAudioEvent.LoadData"); }
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return;
    int n = 0;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn && n < 300; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AkAudioEvent") continue;
        const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
        std::wstring nm = fnm->to_string(); std::wstring lo = nm; for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        if (lo.find(L"weaanim_player") == std::wstring::npos && lo.find(L"wep_") == std::wstring::npos) continue;
        if (lo.find(L"_fire") != std::wstring::npos || lo.find(L"impact") != std::wstring::npos || lo.find(L"projectile") != std::wstring::npos) continue;
        ++n;
        uint32_t* idp = ak_event_id_ptr(o);
        API::get()->log_info("[Halo-CampE-UEVR] WWISE loaded event %ls id=%s%u", nm.c_str(), idp ? "" : "?", idp ? *idp : 0u);
    }
    API::get()->log_info("[Halo-CampE-UEVR] WWISE loaded reload events after the press: %d", n);
    // The game's RTPC / switch / state assets (names), once: a player-only switch or a volume
    // RTPC the sim sets on its emitters would explain accepted-yet-silent posts elsewhere.
    static bool s_groups = false;
    if (!s_groups) {
        s_groups = true;
        std::wstring line; int k = 0;
        for (int32_t i = 0; i < nn && k < 400; ++i) {
            auto* o = static_cast<API::UObject*>(arr->get_object(i));
            if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
            const std::wstring cl = class_name_of(o);
            if (cl != L"AkRtpc" && cl != L"AkSwitchValue" && cl != L"AkStateValue" && cl != L"AkTrigger" && cl != L"AkAuxBus") continue;
            const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
            line += L" " + cl.substr(2) + L":" + fnm->to_string(); ++k;
            if (line.size() > 900) { API::get()->log_info("[Halo-CampE-UEVR] WWISE groups%ls", line.c_str()); line.clear(); }
        }
        API::get()->log_info("[Halo-CampE-UEVR] WWISE groups%ls (%d total)", line.c_str(), k);
    }
}
// ---- THE ENGINE HOOK (see Config.hpp ak_post_rva). --------------------------------------------
using AkPostFn = uint32_t (*)(uint32_t id, uint64_t go, uint32_t flags, void* cb, void* cookie, uint32_t next, void* pext, uint32_t playing);
AkPostFn s_ak_post_orig = nullptr;
int      s_ak_hook_id = -1;
bool     s_ak_hook_tried = false;
thread_local bool t_ak_our_post = false;   // set around the plugin's own posts
std::atomic<int> s_ak_log_left{0};
// The muted id set and window, published for the audio thread.
std::atomic<long long> g_ak_win_until{0};
// The reload engine is ticking. The detours stay installed for the session; with this false they pass
// every call straight through (see ak_engine_released).
std::atomic<bool>      g_ak_engine_on{false};
std::atomic<uint32_t>  g_ak_mute_ids[48];
// Wwise's id of a name: FNV-1 32 over the lowercase bytes (verified against 12 logged ids).
uint32_t ak_fnv(const std::string& name) { uint32_t h = 0x811C9DC5u; for (unsigned char ch : name) { h *= 0x01000193u; h ^= (uint32_t)tolower(ch); } return h; }
std::atomic<int>       g_ak_mute_n{0};
std::atomic<uint64_t>  g_ak_sim_go{0};        // the last game object the sim posted a muted id on
std::atomic<long long> g_ak_sim_go_at{0};
// ---- THE RECIPE (see Config.hpp ak_mimic) ----------------------------------------------------
using AkSetListenersFn = int (*)(uint64_t go, const uint64_t* ids, uint32_t n);
using AkSetSwitchFn    = int (*)(uint32_t group, uint32_t state, uint64_t go);
using AkSetRtpcFn      = int (*)(uint32_t rtpc, float value, uint64_t go, int32_t ms, int32_t curve, bool bypass);
using AkSetPositionFn  = int (*)(uint64_t go, const void* pos, uint8_t flags);
using AkRegisterFn     = int (*)(uint64_t go, const char* name);
using AkUnregisterFn   = int (*)(uint64_t go);
AkSetListenersFn s_ak_setlisteners_orig = nullptr;
AkSetSwitchFn    s_ak_setswitch_orig = nullptr;
AkSetRtpcFn      s_ak_setrtpc_orig = nullptr;
AkSetPositionFn  s_ak_setposition_orig = nullptr;
AkRegisterFn     s_ak_register = nullptr;      // called, not hooked
AkUnregisterFn   s_ak_unregister = nullptr;
struct AkRecipe {
    uint64_t go = 0;
    uint64_t listeners[8] = {0}; uint32_t nlisteners = 0; bool has_listeners = false;
    uint32_t sw_group[6] = {0}, sw_state[6] = {0}; int nsw = 0;
    uint32_t rtpc[8] = {0}; float rtpc_val[8] = {0}; int nrtpc = 0;
    uint8_t  pos[48] = {0}; bool has_pos = false;
};
AkRecipe s_ak_ring[16]; int s_ak_ring_i = 0;
AkRecipe* ak_recipe_for(uint64_t go, bool create) {
    for (auto& r : s_ak_ring) if (r.go == go && go != 0) return &r;
    if (!create) return nullptr;
    AkRecipe& r = s_ak_ring[s_ak_ring_i]; s_ak_ring_i = (s_ak_ring_i + 1) % 16;
    r = AkRecipe{}; r.go = go; return &r;
}
AkRecipe s_ak_template;            // the sim's reload emitter, copied at its (blocked) post
bool     s_ak_template_ok = false;
std::atomic<uint64_t> g_ak_our_go{0};   // our AkComponent's emitter, seen at our own post
std::atomic<uint32_t> g_ak_pending_id{0};  // reloadakmimic 5: a step event waiting for the sim's next emitter
std::atomic<long long> g_ak_pending_at{0};
std::atomic<uint64_t> g_ak_adopted{0};          // mode 7: the sim emitter we hold
std::atomic<uint64_t> g_ak_newest{0};           // the newest sim emitter seen posting (candidate)
std::atomic<uint64_t> g_ak_listener{0};         // the sim's listener id (from its SetListeners calls)
uint8_t  g_ak_head_pos[48] = {0}; std::atomic<bool> g_ak_head_pos_ok{false};
bool ak_window_open() { const long long u = g_ak_win_until.load(std::memory_order_relaxed); return u != 0 && now_ticks() < u; }
int ak_setlisteners_detour(uint64_t go, const uint64_t* ids, uint32_t n) {
    if (!t_ak_our_post && ids != nullptr && n >= 1 && !IsBadReadPtr(ids, 8)) g_ak_listener.store(ids[0], std::memory_order_relaxed);
    if (ak_window_open() && !t_ak_our_post && ids != nullptr && n <= 8) {
        if (auto* r = ak_recipe_for(go, true)) { r->nlisteners = n; for (uint32_t i = 0; i < n; ++i) r->listeners[i] = ids[i]; r->has_listeners = true; }
    }
    return s_ak_setlisteners_orig(go, ids, n);
}
int ak_setswitch_detour(uint32_t group, uint32_t state, uint64_t go) {
    if (ak_window_open() && !t_ak_our_post) { if (auto* r = ak_recipe_for(go, true)) if (r->nsw < 6) { r->sw_group[r->nsw] = group; r->sw_state[r->nsw] = state; ++r->nsw; } }
    return s_ak_setswitch_orig(group, state, go);
}
int ak_setrtpc_detour(uint32_t rtpc, float value, uint64_t go, int32_t ms, int32_t curve, bool bypass) {
    if (ak_window_open() && !t_ak_our_post && go != 0) { if (auto* r = ak_recipe_for(go, true)) if (r->nrtpc < 8) { r->rtpc[r->nrtpc] = rtpc; r->rtpc_val[r->nrtpc] = value; ++r->nrtpc; } }
    return s_ak_setrtpc_orig(rtpc, value, go, ms, curve, bypass);
}
int ak_setposition_detour(uint64_t go, const void* pos, uint8_t flags) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (!t_ak_our_post && pos != nullptr && go != 0 && go == g_ak_listener.load(std::memory_order_relaxed) && !IsBadReadPtr(pos, 48)) {
        memcpy(g_ak_head_pos, pos, 48); g_ak_head_pos_ok.store(true, std::memory_order_relaxed);
        const uint64_t held = g_ak_adopted.load(std::memory_order_relaxed);
        if (g_ak_engine_on.load(std::memory_order_relaxed) && g_cfg.ak_mimic == 7 && held != 0 && s_ak_setposition_orig) { t_ak_our_post = true; s_ak_setposition_orig(held, pos, flags); t_ak_our_post = false; }
    }
    if (ak_window_open() && !t_ak_our_post && pos != nullptr && !IsBadReadPtr(pos, 48)) { if (auto* r = ak_recipe_for(go, true)) { memcpy(r->pos, pos, 48); r->has_pos = true; } }
    return s_ak_setposition_orig(go, pos, flags);
}
AkUnregisterFn s_ak_unregister_orig = nullptr;
std::atomic<uint64_t> g_ak_deferred_unreg{0};   // the sim's reload emitter whose unregister we held back
int ak_unregister_detour(uint64_t go) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    if (g_ak_engine_on.load(std::memory_order_relaxed) && g_cfg.ak_mimic == 7 && !t_ak_our_post && go != 0 && go < 0xFFFFFFull) {
        const uint64_t held = g_ak_adopted.load(std::memory_order_relaxed);
        if (go == held) return 1;   // ours now; the sim thinks it is gone
        const uint64_t newest = g_ak_newest.load(std::memory_order_relaxed);
        if (go == newest && go != held) {
            // The sim is done with its newest emitter: adopt it, release the one we held.
            g_ak_adopted.store(go, std::memory_order_relaxed);
            if (held != 0) s_ak_unregister_orig(held);
            if (g_ak_head_pos_ok.load(std::memory_order_relaxed) && s_ak_setposition_orig) { t_ak_our_post = true; s_ak_setposition_orig(go, g_ak_head_pos, 3); t_ak_our_post = false; }
            if (g_cfg.ak_log) { static int s_said = 0; if (s_said++ < 12) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC7: adopted the sim emitter 0x%llX (released 0x%llX)", (unsigned long long)go, (unsigned long long)held); }
            return 1;
        }
    }
    if (g_cfg.ak_mimic == 6 && ak_window_open() && !t_ak_our_post && go != 0 && go == g_ak_sim_go.load(std::memory_order_relaxed)) {
        g_ak_deferred_unreg.store(go, std::memory_order_relaxed);
        if (g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC6: the sim's unregister of its reload emitter 0x%llX deferred to the window's end", (unsigned long long)go);
        return 1;   // AK_Success, as far as the sim is concerned
    }
    return s_ak_unregister_orig(go);
}
bool ak_hook_one(int rva, const uint8_t* prologue, size_t n, void* detour, void** orig, const char* what) {
    if (rva == 0) return false;
    const uint8_t* exe = reinterpret_cast<const uint8_t*>(GetModuleHandleA(nullptr));
    void* target = (void*)(exe + (uintptr_t)(uint32_t)rva);
    if (IsBadReadPtr(target, n) || memcmp(target, prologue, n) != 0) { API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: %s prologue mismatch at exe+0x%X -- not hooked", what, (unsigned)rva); return false; }
    const int id = API::get()->param()->functions->register_inline_hook(target, detour, orig);
    if (id < 0 || *orig == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: %s register FAILED (id=%d)", what, id); *orig = nullptr; return false; }
    API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: %s hooked at exe+0x%X id=%d", what, (unsigned)rva, id);
    return true;
}
void ak_recipe_hooks_install() {
    static bool s_tried = false;
    if (s_tried) return;
    s_tried = true;
    static const uint8_t P_LSP[16] = {0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x74,0x24,0x10,0x57,0x48,0x83,0xEC,0x20,0x80};   // SetListeners / SetSwitch / SetPosition(inner) share it
    static const uint8_t P_RTPC[16] = {0x48,0x83,0xEC,0x48,0x0F,0xB6,0x44,0x24,0x78,0x88,0x44,0x24,0x30,0x8B,0x44,0x24};
    static const uint8_t P_REG[16] = {0x40,0x53,0x48,0x83,0xEC,0x20,0x80,0x3D,0xA7,0xF5,0xD6,0x02,0x00,0x48,0x8B,0xD9};
    static const uint8_t P_UNREG[16] = {0x40,0x53,0x48,0x83,0xEC,0x20,0x80,0x3D,0xD7,0x97,0xD6,0x02,0x00,0x48,0x8B,0xD9};
    ak_hook_one(g_cfg.ak_fn_setlisteners, P_LSP, 16, (void*)&ak_setlisteners_detour, (void**)&s_ak_setlisteners_orig, "SetListeners");
    ak_hook_one(g_cfg.ak_fn_setswitch, P_LSP, 16, (void*)&ak_setswitch_detour, (void**)&s_ak_setswitch_orig, "SetSwitch");
    ak_hook_one(g_cfg.ak_fn_setrtpc, P_RTPC, 16, (void*)&ak_setrtpc_detour, (void**)&s_ak_setrtpc_orig, "SetRTPCValue");
    ak_hook_one(g_cfg.ak_fn_setposition, P_LSP, 16, (void*)&ak_setposition_detour, (void**)&s_ak_setposition_orig, "SetPosition");
    const uint8_t* exe = reinterpret_cast<const uint8_t*>(GetModuleHandleA(nullptr));
    auto check = [&](int rva, const uint8_t* pro, const char* what) -> void* {
        void* t = (void*)(exe + (uintptr_t)(uint32_t)rva);
        if (IsBadReadPtr(t, 16) || memcmp(t, pro, 16) != 0) { API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: %s prologue mismatch at exe+0x%X -- not used", what, (unsigned)rva); return nullptr; }
        return t;
    };
    s_ak_register = (AkRegisterFn)check(g_cfg.ak_fn_register, P_REG, "RegisterGameObj");
    if (ak_hook_one(g_cfg.ak_fn_unregister, P_UNREG, 16, (void*)&ak_unregister_detour, (void**)&s_ak_unregister_orig, "UnregisterGameObj"))
        s_ak_unregister = s_ak_unregister_orig;   // our own unregisters go straight to the engine
    else
        s_ak_unregister = (AkUnregisterFn)check(g_cfg.ak_fn_unregister, P_UNREG, "UnregisterGameObj");
}
// Our emitters, unregistered a while after their post.
struct AkOurs { uint64_t go; long long at; };
AkOurs s_ak_ours[8]; int s_ak_ours_i = 0; uint64_t s_ak_next_go = 0x7A5E000000000100ull;
void ak_ours_tick() {
    if (s_ak_unregister == nullptr) return;
    const long long t = now_ticks();
    for (auto& o : s_ak_ours) if (o.go != 0 && t - o.at > ms_to_ticks(4000)) { s_ak_unregister(o.go); o.go = 0; }
}
uint32_t ak_post_detour(uint32_t id, uint64_t go, uint32_t flags, void* cb, void* cookie, uint32_t next, void* pext, uint32_t playing) {
    CFG_HOOK_READ;   // off the game thread: see core/config/CfgRead.hpp
    const long long until = g_ak_win_until.load(std::memory_order_relaxed);
    const bool in_window = until != 0 && now_ticks() < until;
    bool muted = false;
    if (in_window && !t_ak_our_post) {
        const int n = g_ak_mute_n.load(std::memory_order_relaxed);
        for (int i = 0; i < n && !muted; ++i) if (g_ak_mute_ids[i].load(std::memory_order_relaxed) == id) muted = true;
    }
    const bool block = muted && g_cfg.reload_ak_mute == 4;
    if (muted) {
        g_ak_sim_go.store(go, std::memory_order_relaxed); g_ak_sim_go_at.store(now_ticks(), std::memory_order_relaxed);
        if (auto* r = ak_recipe_for(go, false)) { s_ak_template = *r; s_ak_template_ok = true; }
    }
    if (t_ak_our_post && go > 0xFFFFFFull) g_ak_our_go.store(go, std::memory_order_relaxed);
    if (!t_ak_our_post && go != 0 && go < 0xFFFFFFull) g_ak_newest.store(go, std::memory_order_relaxed);
    if (in_window && g_cfg.ak_log && s_ak_log_left.fetch_sub(1) > 0) {
        // WHO posted: the return address as an exe offset (the sim's bridge vs the UE integration),
        // and the emitter kind (a small integer is the sim's own numbering, a pointer-sized value
        // is a UE AkComponent) -- 2026-09-05, to learn how the sim makes its posts audible.
        const uint8_t* exe = reinterpret_cast<const uint8_t*>(GetModuleHandleA(nullptr));
        const uint8_t* ra = reinterpret_cast<const uint8_t*>(_ReturnAddress());
        const long long off = (exe != nullptr) ? (long long)(ra - exe) : -1;
        API::get()->log_info("[Halo-CampE-UEVR] AKPOST id=%u go=0x%llX flags=0x%X cb=%p cookie=%p ext=%u ret=exe+0x%llX %s %s", id, (unsigned long long)go, flags, cb, cookie, next, off,
                             go > 0xFFFFFFull ? "[ptr-emitter]" : "[sim-emitter]",
                             t_ak_our_post ? "OURS" : (block ? "BLOCKED" : (muted ? "known (mode<4, passed)" : "other")));
        if (g_cfg.ak_stack && muted && !t_ak_our_post) {
            void* frames[10] = {0};
            const USHORT n = RtlCaptureStackBackTrace(1, 10, frames, nullptr);
            const uint8_t* sim = reinterpret_cast<const uint8_t*>(GetModuleHandleA("HaloSimulation_tag_release.dll"));
            std::string line;
            for (USHORT i = 0; i < n; ++i) {
                const uint8_t* f = reinterpret_cast<const uint8_t*>(frames[i]);
                char buf[64];
                if (exe != nullptr && f >= exe && f < exe + 0x10000000ull) snprintf(buf, sizeof(buf), " exe+0x%llX", (unsigned long long)(f - exe));
                else if (sim != nullptr && f >= sim && f < sim + 0x4000000ull) snprintf(buf, sizeof(buf), " sim+0x%llX", (unsigned long long)(f - sim));
                else snprintf(buf, sizeof(buf), " %p", (const void*)f);
                line += buf;
            }
            API::get()->log_info("[Halo-CampE-UEVR] AKSTACK id=%u:%s", id, line.c_str());
        }
    }
    if (!t_ak_our_post && (in_window || g_cfg.ak_mimic == 7) && (g_cfg.ak_mimic == 5 || g_cfg.ak_mimic == 6 || g_cfg.ak_mimic == 7)) {
        const uint32_t pend = g_ak_pending_id.load(std::memory_order_relaxed);
        if (pend != 0 && now_ticks() - g_ak_pending_at.load(std::memory_order_relaxed) < ms_to_ticks(1500)) {
            g_ak_pending_id.store(0, std::memory_order_relaxed);
            const uint32_t pl = s_ak_post_orig(pend, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
            if (g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC5: step event %u rode the sim's emitter 0x%llX (its %u) -> playing %u", pend, (unsigned long long)go, id, pl);
        }
    }
    if (block) {
        if (g_cfg.ak_mimic == 4) {
            const std::string sub = trim_cfg(g_cfg.ak_mimic4_event);
            const uint32_t post_id = sub.empty() ? id : ak_fnv(sub);
            const uint32_t pl = s_ak_post_orig(post_id, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
            if (g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC4: in place of the sim's %u, %u ('%s') posted by the plugin on 0x%llX, flags 0 -> playing %u", id, post_id, sub.c_str(), (unsigned long long)go, pl);
            return pl;
        }
        return 0;   // AK_INVALID_PLAYING_ID
    }
    return s_ak_post_orig(id, go, flags, cb, cookie, next, pext, playing);
}
void ak_hook_install() {
    if (s_ak_hook_tried) return;
    s_ak_hook_tried = true;
    if (g_cfg.ak_post_rva == 0) return;
    const uint8_t* exe = reinterpret_cast<const uint8_t*>(GetModuleHandleA(nullptr));
    if (exe == nullptr) return;
    void* target = (void*)(exe + (uintptr_t)(uint32_t)g_cfg.ak_post_rva);
    static const uint8_t PROLOGUE[16] = {0x48,0x8B,0xC4,0x48,0x89,0x58,0x08,0x48,0x89,0x68,0x10,0x48,0x89,0x70,0x18,0x4C};
    if (IsBadReadPtr(target, sizeof(PROLOGUE)) || memcmp(target, PROLOGUE, sizeof(PROLOGUE)) != 0) {
        API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: prologue mismatch at exe+0x%X -- the game moved; NOT hooking", (unsigned)g_cfg.ak_post_rva);
        return;
    }
    const int id = API::get()->param()->functions->register_inline_hook(target, (void*)&ak_post_detour, (void**)&s_ak_post_orig);
    if (id < 0 || s_ak_post_orig == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: register_inline_hook FAILED (id=%d)", id); s_ak_post_orig = nullptr; return; }
    s_ak_hook_id = id;
    API::get()->log_info("[Halo-CampE-UEVR] AKHOOK: PostEvent hooked at exe+0x%X id=%d", (unsigned)g_cfg.ak_post_rva, id);
}
void ak_set_rtpc(API::UObject* actor, float value, const char* why);   // defined with the step posts below
// reloadakmute: the weapon's reload events with their ids zeroed for the window.
struct AkMuted { TrackedObject ev; uint32_t id; };
AkMuted s_akm[24]; int s_akm_n = 0;
long long s_akm_until = 0;
// The set is remembered per weapon stem: the first press pays the object-array walk, the rest
// re-check the handles (one slot compare each). Up to 25 walks per press before (audit, 2026-09-06).
std::string s_akm_cache_stem; AkMuted s_akm_cache[24]; int s_akm_cache_n = 0;
const char* ak_weapon_token(const std::string& stem) {
    if (stem.rfind("magnum", 0) == 0) return "magnum_";
    if (stem.rfind("assaultrifle", 0) == 0) return "ar_";
    if (stem.rfind("smg", 0) == 0) return "ar_";   // the SMG reloads with the AR's events (post log, 2026-09-06)
    if (stem.rfind("battlerifle", 0) == 0) return "br_";
    if (stem.rfind("sniperrifle", 0) == 0) return "sniperrifle_";
    if (stem.rfind("needler", 0) == 0) return "needler_";
    if (stem.rfind("rocketlauncher", 0) == 0) return "spnker_rocket_launcher_";
    if (stem.rfind("plasmapistol", 0) == 0) return "plasmapistol_";
    if (stem.rfind("shotgun", 0) == 0) return "wep_fol_shotgun_";
    if (stem.rfind("spikerifle", 0) == 0) return "wep_spikerifle_reload";
    if (stem.rfind("fuelrodcannon", 0) == 0) return "fuelrodgun_reload";
    return nullptr;
}
void ak_id_mute_begin() {
    if (g_cfg.reload_ak_mute == 0 || g_cfg.reload_mute_ms <= 0) return;
    if (s_akm_n > 0) return;   // a window is already open
    std::string stem; { const std::wstring w = ak_weapon_stem(); stem.assign(w.begin(), w.end()); }
    const char* tok = ak_weapon_token(stem);
    if (tok == nullptr) {
        // A weapon with no known reload events (the SMG, 2026-09-06): the window still opens with
        // nothing to refuse, so the post log shows what the game plays for it.
        s_akm_until = now_ticks() + ms_to_ticks(g_cfg.reload_mute_ms);
        g_ak_mute_n.store(0, std::memory_order_relaxed);
        g_ak_win_until.store(s_akm_until, std::memory_order_relaxed);
        s_ak_log_left.store(60, std::memory_order_relaxed);
        if (g_cfg.reload_ak_mute == 4 || g_cfg.ak_log) ak_hook_install();
        if (g_cfg.reload_vr_log || g_cfg.reload_wwise_dump || g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] WWISE mute: no reload events known for '%s', window open for the log only", stem.c_str());
        return;
    }
    std::wstring wtok(tok, tok + strlen(tok));
    const bool chm = wtok.find(L"wep_") == std::wstring::npos && wtok.find(L"fuelrod") == std::wstring::npos;
    const std::wstring t_player = chm ? (L"weaanim_player_" + wtok) : wtok;
    const std::wstring t_nonplayer = chm ? (L"weaanim_nonplayer_" + wtok) : (wtok + L"_nonplayer");
    bool from_cache = false;
    if (s_akm_cache_n > 0 && s_akm_cache_stem == stem) {
        from_cache = true;
        for (int i = 0; i < s_akm_cache_n; ++i) if (s_akm_cache[i].ev.get() == nullptr) { from_cache = false; break; }
        if (from_cache) {
            for (int i = 0; i < s_akm_cache_n; ++i) {
                s_akm[i] = s_akm_cache[i];
                if (auto* o = s_akm[i].ev.get()) {
                    if (g_cfg.reload_ak_mute == 1) { if (auto* idp = ak_event_id_ptr(o)) *idp = 0; }
                    else if (g_cfg.reload_ak_mute == 2) { alignas(16) uint8_t p[32] = {0}; o->call_function(L"UnloadData", p); }
                }
            }
            s_akm_n = s_akm_cache_n;
        }
    }
    if (!from_cache) {
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn && s_akm_n < 24; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AkAudioEvent") continue;
        const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
        std::wstring lo = fnm->to_string(); for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        const bool is_player = lo.find(t_player) != std::wstring::npos;
        const bool is_nonplayer = lo.find(t_nonplayer) != std::wstring::npos || (chm && lo.find(L"nonplayer") != std::wstring::npos && lo.find(wtok) != std::wstring::npos);
        if (!(is_player || is_nonplayer)) continue;
        if (g_cfg.reload_mute_variant == 0 && !is_player) continue;
        if (g_cfg.reload_mute_variant == 1 && !is_nonplayer) continue;
        if (lo.find(L"_fire") != std::wstring::npos || lo.find(L"dryfire") != std::wstring::npos) continue;
        uint32_t* idp = ak_event_id_ptr(o); if (idp == nullptr || *idp == 0) continue;
        s_akm[s_akm_n].ev.set_at(o, i); s_akm[s_akm_n].id = *idp; ++s_akm_n;
        if (g_cfg.reload_wwise_dump) API::get()->log_info("[Halo-CampE-UEVR] WWISE mute covers %ls (id %u)", fnm->to_string().c_str(), *idp);
        if (g_cfg.reload_ak_mute == 1) *idp = 0;
        else if (g_cfg.reload_ak_mute == 2) { alignas(16) uint8_t p[32] = {0}; o->call_function(L"UnloadData", p); }
    }
    s_akm_cache_stem = stem; s_akm_cache_n = s_akm_n;
    for (int i = 0; i < s_akm_n; ++i) s_akm_cache[i] = s_akm[i];
    }
    s_akm_until = now_ticks() + ms_to_ticks(g_cfg.reload_mute_ms);
    int nid = 0;
    for (int i = 0; i < s_akm_n && nid < 48; ++i) g_ak_mute_ids[nid++].store(s_akm[i].id, std::memory_order_relaxed);
    {   // the foley names, hashed
        const std::string lst = trim_cfg(g_cfg.ak_mute_names);
        size_t pos = 0;
        while (pos < lst.size() && nid < 48) {
            size_t comma = lst.find(',', pos); if (comma == std::string::npos) comma = lst.size();
            std::string n = lst.substr(pos, comma - pos);
            while (!n.empty() && (unsigned char)n.back() <= ' ') n.pop_back();
            while (!n.empty() && (unsigned char)n.front() <= ' ') n.erase(n.begin());
            if (!n.empty()) g_ak_mute_ids[nid++].store(ak_fnv(n), std::memory_order_relaxed);
            pos = comma + 1;
        }
    }
    g_ak_mute_n.store(nid, std::memory_order_relaxed);
    g_ak_win_until.store(s_akm_until, std::memory_order_relaxed);
    s_ak_log_left.store(60, std::memory_order_relaxed);
    if (g_cfg.reload_ak_mute == 4 || g_cfg.ak_log) ak_hook_install();
    if (g_cfg.ak_mimic != 0) ak_recipe_hooks_install();
    if (g_cfg.ak_rtpc_global) ak_set_rtpc(nullptr, g_cfg.ak_rtpc_value, "window open");
    if (g_cfg.reload_vr_log || g_cfg.reload_wwise_dump) API::get()->log_info("[Halo-CampE-UEVR] WWISE mute mode %d: %d reload event(s) of '%s' for %d ms", g_cfg.reload_ak_mute, s_akm_n, stem.c_str(), g_cfg.reload_mute_ms);
}
void ak_id_mute_end() {
    for (int i = 0; i < s_akm_n; ++i) {
        if (auto* o = s_akm[i].ev.get()) {
            if (g_cfg.reload_ak_mute == 1) { if (auto* idp = ak_event_id_ptr(o)) *idp = s_akm[i].id; }
            else if (g_cfg.reload_ak_mute == 2) { alignas(16) uint8_t p[32] = {0}; o->call_function(L"LoadData", p); }
        }
        s_akm[i] = AkMuted{};
    }
    if (s_akm_n > 0 && (g_cfg.reload_vr_log || g_cfg.reload_wwise_dump)) API::get()->log_info("[Halo-CampE-UEVR] WWISE mute: %d event(s) restored", s_akm_n);
    if (s_ak_template_ok && g_cfg.ak_log) {
        const AkRecipe& r = s_ak_template;
        std::string l; for (uint32_t i = 0; i < r.nlisteners; ++i) { char b2[32]; snprintf(b2, sizeof(b2), " 0x%llX", (unsigned long long)r.listeners[i]); l += b2; }
        std::string sw; for (int i = 0; i < r.nsw; ++i) { char b2[48]; snprintf(b2, sizeof(b2), " %u=%u", r.sw_group[i], r.sw_state[i]); sw += b2; }
        std::string rt; for (int i = 0; i < r.nrtpc; ++i) { char b2[48]; snprintf(b2, sizeof(b2), " %u=%.2f", r.rtpc[i], r.rtpc_val[i]); rt += b2; }
        const float* pf = reinterpret_cast<const float*>(r.pos);
        API::get()->log_info("[Halo-CampE-UEVR] AKRECIPE go=0x%llX listeners(%u):%s switches:%s rtpcs:%s pos=%s[%.2f %.2f %.2f | %.2f %.2f %.2f | %.1f %.1f %.1f]", (unsigned long long)r.go, r.nlisteners, l.c_str(), sw.c_str(), rt.c_str(), r.has_pos ? "" : "(none)", pf[0], pf[1], pf[2], pf[3], pf[4], pf[5], pf[6], pf[7], pf[8]);
    }
    s_akm_n = 0; s_akm_until = 0;
    g_ak_win_until.store(0, std::memory_order_relaxed); g_ak_mute_n.store(0, std::memory_order_relaxed);
    {   // the sim's reload emitter we kept alive goes now
        const uint64_t d = g_ak_deferred_unreg.exchange(0, std::memory_order_relaxed);
        if (d != 0 && s_ak_unregister_orig != nullptr) { t_ak_our_post = true; s_ak_unregister_orig(d); t_ak_our_post = false; if (g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC6: reload emitter 0x%llX unregistered at the window's end", (unsigned long long)d); }
        g_ak_sim_go.store(0, std::memory_order_relaxed);
    }
    if (g_cfg.ak_rtpc_global) ak_set_rtpc(nullptr, g_cfg.ak_rtpc_restore, "window closed");
}
// Mode 3: the game's post is cut every tick of the window, on OUR firearm's emitter and its
// owner's (the pawn) only -- from the headset, 2026-09-05: only our firearm's animation sound is muted --
// except an event the plugin itself posted in the last 1500 ms.
struct AkPosted { TrackedObject ev; long long at; };
AkPosted s_ak_posted[8]; int s_ak_posted_i = 0;
void ak_note_posted(API::UObject* ev) { s_ak_posted[s_ak_posted_i].ev.set(ev); s_ak_posted[s_ak_posted_i].at = now_ticks(); s_ak_posted_i = (s_ak_posted_i + 1) % 8; }
bool ak_posted_recently(API::UObject* ev) { const long long t = now_ticks(); for (auto& p : s_ak_posted) if (p.ev.get() == ev && t - p.at < ms_to_ticks(1500)) return true; return false; }
void ak_mute_stop_tick() {
    if (g_cfg.reload_ak_mute != 3 || s_akm_n == 0) return;
    auto* wpn = fp_weapon_actor();
    if (wpn == nullptr) return;
    API::UObject* owner = nullptr;
    { alignas(16) uint8_t p[64] = {0}; wpn->call_function(L"GetOwner", p); owner = *reinterpret_cast<API::UObject**>(p); }
    for (int i = 0; i < s_akm_n; ++i) {
        auto* o = s_akm[i].ev.get();
        if (o == nullptr || ak_posted_recently(o)) continue;
        ak_event_stop(o, wpn);
        if (owner != nullptr) ak_event_stop(o, owner);
    }
}
// A muted event's real id, for the plugin's own post: restored around the call.
uint32_t ak_muted_id_of(API::UObject* ev) { for (int i = 0; i < s_akm_n; ++i) if (s_akm[i].ev.get() == ev) return s_akm[i].id; return 0; }
void ak_bus_volume(float vol) {
    auto* arr = API::get()->get_uobject_array();
    auto* wpn = fp_weapon_actor();
    if (arr == nullptr || wpn == nullptr) return;
    API::UObject* wpn_owner = nullptr;
    { alignas(16) uint8_t p[64] = {0}; wpn->call_function(L"GetOwner", p); wpn_owner = *reinterpret_cast<API::UObject**>(p); }
    int n = 0;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AkComponent") continue;
        API::UObject* owner = nullptr;
        { alignas(16) uint8_t p[64] = {0}; o->call_function(L"GetOwner", p); owner = *reinterpret_cast<API::UObject**>(p); }
        if (owner == nullptr || (owner != wpn && (wpn_owner == nullptr || owner != wpn_owner))) continue;
        alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<float*>(p) = vol;
        o->call_function(L"SetOutputBusVolume", p);
        ++n;
    }
    if (g_cfg.reload_vr_log || g_cfg.reload_wwise_dump) API::get()->log_info("[Halo-CampE-UEVR] WWISE mute: bus volume %.1f on %d AkComponent(s) of the weapon/owner", vol, n);
}
// AKVTDUMP: the engine interface's vtable, once (see Config.hpp ak_vt_dump).
void ak_vt_dump() {
    static bool s_done = false;
    if (!g_cfg.ak_vt_dump || s_done) return;
    s_done = true;
    const uint8_t* exe = reinterpret_cast<const uint8_t*>(GetModuleHandleA(nullptr));
    if (exe == nullptr) return;
    const uint8_t* const* gp = reinterpret_cast<const uint8_t* const*>(exe + (uintptr_t)(uint32_t)g_cfg.ak_vt_global);
    if (IsBadReadPtr(gp, sizeof(void*))) { API::get()->log_info("[Halo-CampE-UEVR] AKVT: global exe+0x%X unreadable", (unsigned)g_cfg.ak_vt_global); return; }
    const uint8_t* obj = *gp;
    if (obj == nullptr || IsBadReadPtr(obj, sizeof(void*))) { API::get()->log_info("[Halo-CampE-UEVR] AKVT: object null at exe+0x%X", (unsigned)g_cfg.ak_vt_global); return; }
    const uint8_t* const* vt = *reinterpret_cast<const uint8_t* const* const*>(obj);
    if (vt == nullptr || IsBadReadPtr(vt, sizeof(void*))) { API::get()->log_info("[Halo-CampE-UEVR] AKVT: vtable unreadable"); return; }
    API::get()->log_info("[Halo-CampE-UEVR] AKVT: exe %p object %p vtable rva 0x%llX", (const void*)exe, (const void*)obj, (unsigned long long)(reinterpret_cast<const uint8_t*>(vt) - exe));
    std::string line;
    for (int i = 0; i < g_cfg.ak_vt_count; ++i) {
        if (IsBadReadPtr(vt + i, sizeof(void*))) break;
        const uint8_t* f = vt[i];
        if (f == nullptr) break;
        char buf[48]; snprintf(buf, sizeof(buf), " %d:0x%llX", i, (unsigned long long)(f - exe));
        line += buf;
        if (line.size() > 900) { API::get()->log_info("[Halo-CampE-UEVR] AKVT slots%s", line.c_str()); line.clear(); }
    }
    if (!line.empty()) API::get()->log_info("[Halo-CampE-UEVR] AKVT slots%s", line.c_str());
}
void ak_mute_begin() {
    ak_vt_dump();
    ak_id_mute_begin();
    ak_dump_after_press();
}
void ak_mute_tick() {
    if (g_cfg.ak_mimic != 0 || g_cfg.reload_ak_mute == 4 || g_cfg.ak_log) { ak_hook_install(); if (g_cfg.ak_mimic != 0) ak_recipe_hooks_install(); }
    ak_dump_after_press_tick();
    ak_ours_tick();
    if (s_akm_until == 0) return;
    if (now_ticks() < s_akm_until) { ak_mute_stop_tick(); return; }
    ak_id_mute_end();
}
// BOTH RELOAD FEATURES SWITCHED OFF (reload_engine_released, game thread). The Wwise detours stay
// installed -- removing an inline hook while the audio thread may be inside it is the riskier move -- but
// stand down: the flag above makes the reloadakmimic 7 adoption and head-position push pass through, a mute
// window still open is closed now (restoring the events it muted, which only the engine's own tick did),
// and the sim emitter reloadakmimic 7 was holding goes back to the engine. A detour already past the flag test
// on another thread can still adopt one emitter; it is released the next time reload comes on and off.
void ak_engine_released() {
    g_ak_engine_on.store(false, std::memory_order_relaxed);
    if (s_akm_n > 0 || s_akm_until != 0 || g_ak_win_until.load(std::memory_order_relaxed) != 0) ak_id_mute_end();
    const uint64_t held = g_ak_adopted.exchange(0, std::memory_order_relaxed);
    if (held != 0 && s_ak_unregister_orig != nullptr) {
        t_ak_our_post = true; s_ak_unregister_orig(held); t_ak_our_post = false;
        if (g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC7: released the held sim emitter 0x%llX (reload switched off)", (unsigned long long)held);
    }
    g_ak_newest.store(0, std::memory_order_relaxed);
}
// reloadstepsound: "Weapon:drop=Ev,seat=Ev,rack=Ev;*:drop=Ev" -- the event for this weapon and step.
std::string ak_step_event_in(const char* table, const char* step) {
    std::string tbl = trim_cfg(table);
    if (tbl.empty()) return "";
    std::string stem; { const std::wstring w = ak_weapon_stem(); stem.assign(w.begin(), w.end()); }
    std::string fallback;
    size_t pos = 0;
    while (pos < tbl.size()) {
        size_t semi = tbl.find(';', pos); if (semi == std::string::npos) semi = tbl.size();
        std::string ent = tbl.substr(pos, semi - pos);
        const size_t colon = ent.find(':');
        if (colon != std::string::npos) {
            std::string w = ent.substr(0, colon), rest = ent.substr(colon + 1);
            for (auto& ch : w) ch = (char)tolower((unsigned char)ch);
            std::string found;
            size_t q = 0;
            while (q < rest.size()) {
                size_t comma = rest.find(',', q); if (comma == std::string::npos) comma = rest.size();
                std::string kv = rest.substr(q, comma - q);
                const size_t eq = kv.find('=');
                if (eq != std::string::npos && _stricmp(kv.substr(0, eq).c_str(), step) == 0) found = kv.substr(eq + 1);
                q = comma + 1;
            }
            while (!found.empty() && (unsigned char)found.back() <= ' ') found.pop_back();
            if (stem.rfind(w, 0) == 0 && !found.empty()) return found;
            if (w == "*" && !found.empty()) fallback = found;
        }
        pos = semi + 1;
    }
    return fallback;
}
std::string ak_step_event(const char* step) {
    const std::string o = ak_step_event_in(g_cfg.reload_step_override, step);
    if (!o.empty()) return (o == "none") ? "" : o;
    return ak_step_event_in(g_cfg.reload_step_sound, step);
}
API::UObject* ak_find_event(const std::string& name_in) {
    std::string name = name_in;
    if (name.rfind("Play_", 0) != 0 && name.rfind("play_", 0) != 0) name = "Play_006_chm_ge_weaanim_player_" + name;
    for (auto& ch : name) ch = (char)tolower((unsigned char)ch);
    if (g_cfg.reload_step_variant == 1) { const size_t at = name.find("weaanim_player_"); if (at != std::string::npos) name.replace(at, 15, "weaanim_nonplayer_"); }
    // The table: every step name the session has asked for, hit or miss. The old one-entry cache
    // thrashed between alternating steps and never remembered a miss, so every rack walked the
    // whole object array (perf audit, 2026-09-06). A miss is retried after 20 s.
    struct AkEvCache { std::string name; TrackedObject obj; long long at; bool found; };
    static AkEvCache s_tab[32]; static int s_tab_n = 0;
    const long long nowt = now_ticks();
    AkEvCache* slot = nullptr;
    for (int i = 0; i < s_tab_n; ++i) if (s_tab[i].name == name) { slot = &s_tab[i]; break; }
    if (slot != nullptr) {
        if (slot->found) { if (auto* o = slot->obj.get()) return o; }
        else if (nowt - slot->at < ms_to_ticks(20000)) return nullptr;
    } else {
        slot = &s_tab[s_tab_n < 32 ? s_tab_n++ : 31];
        slot->name = name;
    }
    slot->found = false; slot->at = nowt; slot->obj = TrackedObject{};
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return nullptr;
    std::wstring want(name.begin(), name.end());
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AkAudioEvent") continue;
        const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
        std::wstring lo = fnm->to_string(); for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        if (lo == want) { slot->obj.set_at(o, i); slot->found = true; return o; }
    }
    return nullptr;
}
// The RTPC (see Config.hpp ak_rtpc): the AkRtpc asset by name, set on an actor (or globally).
API::UObject* ak_find_rtpc(const std::string& name) {
    static std::string s_name; static TrackedObject s_obj;
    if (name == s_name) if (auto* o = s_obj.get()) return o;
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return nullptr;
    std::wstring want(name.begin(), name.end());
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"AkRtpc") continue;
        const auto* fnm = o->get_fname(); if (fnm == nullptr) continue;
        if (fnm->to_string() == want) { s_name = name; s_obj.set(o); return o; }
    }
    return nullptr;
}
void ak_set_rtpc(API::UObject* actor, float value, const char* why) {
    const std::string nm = trim_cfg(g_cfg.ak_rtpc);
    if (nm.empty()) return;
    auto* rtpc = ak_find_rtpc(nm);
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/AkAudio.AkGameplayStatics");
    auto* fn = cls ? cls->find_function(L"SetRTPCValue") : nullptr;
    auto* cdo = cls ? cls->get_class_default_object() : nullptr;
    static bool s_sig = false;
    if (!s_sig) { s_sig = true; ak_dump_function(L"Function /Script/AkAudio.AkGameplayStatics.SetRTPCValue"); }
    if (rtpc == nullptr || fn == nullptr || cdo == nullptr) { static int s_said = 0; if (s_said++ < 3) API::get()->log_info("[Halo-CampE-UEVR] AKRTPC '%s': asset %s, SetRTPCValue %s", nm.c_str(), rtpc ? "found" : "NOT LOADED", fn ? "ok" : "NOT FOUND"); return; }
    alignas(16) uint8_t p[256] = {0};
    if ((size_t)fn->get_properties_size() > sizeof(p)) return;
    auto put = [&](const wchar_t* name, const void* v, size_t n) { auto* pr = fn->find_property(name); if (pr == nullptr) return false; const int32_t off = pr->get_offset(); if (off >= 0 && (size_t)off + n <= sizeof(p)) memcpy(p + off, v, n); return true; };
    const int32_t interp = 0;
    put(L"RTPCValue", &rtpc, sizeof(void*));
    put(L"Value", &value, sizeof(float));
    put(L"InterpolationTimeMs", &interp, sizeof(int32_t));
    put(L"Actor", &actor, sizeof(void*));
    cdo->call_function(L"SetRTPCValue", p);
    if (g_cfg.reload_vr_log || g_cfg.reload_wwise_dump) API::get()->log_info("[Halo-CampE-UEVR] AKRTPC '%s' = %.1f on %ls (%s)", nm.c_str(), value, actor ? class_name_of(actor).c_str() : L"GLOBAL", why);
}
void ak_step_sound(const char* step) {
    std::string ev;
    if (s_sl_empty_at_drop) ev = ak_step_event((std::string(step) + "empty").c_str());
    if (ev.empty()) ev = ak_step_event(step);
    if (ev.empty()) return;
    auto* evo = ak_find_event(ev);
    auto* wpn = fp_weapon_actor();
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/AkAudio.AkGameplayStatics");
    auto* fn = cls ? cls->find_function(L"PostEvent") : nullptr;
    auto* cdo = cls ? cls->get_class_default_object() : nullptr;
    if (evo == nullptr && g_cfg.ak_mimic == 7 && s_ak_post_orig != nullptr) {
        // Not loaded yet (the game loads an event's object on its first play): the id is the
        // FNV-1 of the name, and the adopted emitter takes it as it would any other.
        const uint64_t go = g_ak_adopted.load(std::memory_order_relaxed);
        const uint32_t evid = ak_fnv(ev);
        if (go != 0) {
            const uint32_t pl = s_ak_post_orig(evid, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
            if (g_cfg.reload_vr_log || g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC7 step %s: '%s' has no loaded object, posted by hash %u on 0x%llX -> playing %u", step, ev.c_str(), evid, (unsigned long long)go, pl);
            return;
        }
    }
    if (evo == nullptr || wpn == nullptr || fn == nullptr || cdo == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: event '%s' %s, weapon %s, PostEvent %s", step, ev.c_str(), evo ? "found" : "NOT FOUND", wpn ? "ok" : "none", fn ? "ok" : "NOT FOUND");
        return;
    }
    alignas(16) uint8_t p[256] = {0};
    const size_t need = (size_t)fn->get_properties_size();
    if (need > sizeof(p)) { API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: PostEvent params 0x%X too big", step, (unsigned)need); return; }
    bool ok = true;
    auto put = [&](const wchar_t* name, const void* v, size_t n) {
        auto* pr = fn->find_property(name);
        if (pr == nullptr) { ok = false; API::get()->log_info("[Halo-CampE-UEVR] WWISE step: PostEvent has no '%ls'", name); return; }
        const int32_t off = pr->get_offset();
        if (off < 0 || (size_t)off + n > sizeof(p)) { ok = false; return; }
        memcpy(p + off, v, n);
    };
    // The actor the post rides: the weapon, or its owner (the pawn) in mode 2.
    API::UObject* on = wpn;
    if (g_cfg.reload_step_via == 2) {
        alignas(16) uint8_t q[64] = {0}; wpn->call_function(L"GetOwner", q);
        if (auto* o = *reinterpret_cast<API::UObject**>(q)) on = o;
    }
    put(L"AkEvent", &evo, sizeof(void*));
    put(L"Actor", &on, sizeof(void*));
    if (!ok) return;
    if (!g_cfg.ak_rtpc_global && g_cfg.reload_step_via != 3 && g_cfg.reload_step_via != 4) ak_set_rtpc(on, g_cfg.ak_rtpc_value, step);
    // A muted event posts with its real id for this one call.
    const uint32_t muted = (g_cfg.reload_ak_mute == 1) ? ak_muted_id_of(evo) : 0;
    uint32_t* idp = muted ? ak_event_id_ptr(evo) : nullptr;
    if (idp != nullptr) *idp = muted;
    uint32_t playing = 0;
    ak_note_posted(evo);
    t_ak_our_post = true;
    Vec3 gun{}; bool have_gun = false;
    bool posted = false;
    if (g_cfg.ak_mimic == 1 && s_ak_template_ok && s_ak_post_orig != nullptr && s_ak_register != nullptr) {
        // OUR OWN EMITTER, the sim's recipe: register, position, listeners, switches, RTPCs, post.
        const AkRecipe& r = s_ak_template;
        const uint64_t go = s_ak_next_go++;
        const uint32_t evid = ak_event_short_id(evo);
        int rr = s_ak_register(go, "halo_vr_reload");
        int rp = -1, rl = -1;
        if (r.has_pos && s_ak_setposition_orig) rp = s_ak_setposition_orig(go, r.pos, 3);
        if (r.has_listeners && s_ak_setlisteners_orig) rl = s_ak_setlisteners_orig(go, r.listeners, r.nlisteners);
        if (s_ak_setswitch_orig) for (int i = 0; i < r.nsw; ++i) s_ak_setswitch_orig(r.sw_group[i], r.sw_state[i], go);
        if (s_ak_setrtpc_orig) for (int i = 0; i < r.nrtpc; ++i) s_ak_setrtpc_orig(r.rtpc[i], r.rtpc_val[i], go, 0, 4, false);
        playing = s_ak_post_orig(evid, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
        s_ak_ours[s_ak_ours_i] = AkOurs{go, now_ticks()}; s_ak_ours_i = (s_ak_ours_i + 1) % 8;
        posted = true;
        API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC step %s: emitter 0x%llX register=%d pos=%d listeners=%d(%u) switches=%d rtpcs=%d -> post %u playing %u", step, (unsigned long long)go, rr, rp, rl, r.nlisteners, r.nsw, r.nrtpc, evid, playing);
    } else if (g_cfg.ak_mimic == 7 && s_ak_post_orig != nullptr) {
        const uint32_t evid = ak_event_short_id(evo);
        const uint64_t go = g_ak_adopted.load(std::memory_order_relaxed);
        if (go != 0) {
            playing = s_ak_post_orig(evid, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
            posted = true;
            if (g_cfg.reload_vr_log || g_cfg.ak_log) API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC7 step %s: event %u on the adopted emitter 0x%llX -> playing %u", step, evid, (unsigned long long)go, playing);
        } else {
            g_ak_pending_id.store(evid, std::memory_order_relaxed); g_ak_pending_at.store(now_ticks(), std::memory_order_relaxed);
            posted = true;
            API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC7 step %s: nothing adopted yet, event %u parked for the next sim emitter", step, evid);
        }
    } else if (g_cfg.ak_mimic == 6 && s_ak_post_orig != nullptr) {
        const uint32_t evid = ak_event_short_id(evo);
        const uint64_t go = g_ak_sim_go.load(std::memory_order_relaxed);
        if (go != 0 && ak_window_open()) {
            playing = s_ak_post_orig(evid, go, 0u, nullptr, nullptr, 0u, nullptr, 0u);
            posted = true;
            API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC6 step %s: event %u on the sim's reload emitter 0x%llX -> playing %u", step, evid, (unsigned long long)go, playing);
        } else {
            g_ak_pending_id.store(evid, std::memory_order_relaxed); g_ak_pending_at.store(now_ticks(), std::memory_order_relaxed);
            posted = true;
            API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC6 step %s: no reload emitter yet, event %u parked for the sim's next", step, evid);
        }
    } else if (g_cfg.ak_mimic == 5) {
        const uint32_t evid = ak_event_short_id(evo);
        g_ak_pending_id.store(evid, std::memory_order_relaxed); g_ak_pending_at.store(now_ticks(), std::memory_order_relaxed);
        posted = true;
        API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC5 step %s: event %u parked for the sim's next emitter", step, evid);
    } else if ((g_cfg.ak_mimic == 2 || g_cfg.ak_mimic == 3) && s_ak_template_ok) {
        // OUR COMPONENT'S EMITTER (learned at the previous post), given the sim's listeners (2) or
        // its switches and RTPCs (3) before the normal post below.
        const uint64_t ours = g_ak_our_go.load(std::memory_order_relaxed);
        const AkRecipe& r = s_ak_template;
        if (ours != 0) {
            if (g_cfg.ak_mimic == 2 && r.has_listeners && s_ak_setlisteners_orig) { const int rl = s_ak_setlisteners_orig(ours, r.listeners, r.nlisteners); API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC step %s: listeners(%u) on our emitter 0x%llX -> %d", step, r.nlisteners, (unsigned long long)ours, rl); }
            if (g_cfg.ak_mimic == 3) { if (s_ak_setswitch_orig) for (int i = 0; i < r.nsw; ++i) s_ak_setswitch_orig(r.sw_group[i], r.sw_state[i], ours); if (s_ak_setrtpc_orig) for (int i = 0; i < r.nrtpc; ++i) s_ak_setrtpc_orig(r.rtpc[i], r.rtpc_val[i], ours, 0, 4, false); API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC step %s: %d switch(es), %d rtpc(s) on our emitter 0x%llX", step, r.nsw, r.nrtpc, (unsigned long long)ours); }
        } else API::get()->log_info("[Halo-CampE-UEVR] AKMIMIC step %s: our emitter not seen yet (this post teaches it)", step);
    }
    if (!posted && g_cfg.reload_step_via == 4 && s_ak_post_orig != nullptr) {
        // THE SIM'S EMITTER (see Config.hpp): only once the sim has posted in this window.
        const long long at = g_ak_sim_go_at.load(std::memory_order_relaxed);
        const uint64_t go = g_ak_sim_go.load(std::memory_order_relaxed);
        if (go != 0 && at != 0 && now_ticks() - at < ms_to_ticks(g_cfg.reload_mute_ms)) {
            const uint32_t id = ak_event_short_id(evo);
            if (id != 0) { playing = s_ak_post_orig(id, go, 1u, nullptr, nullptr, 0u, nullptr, 0u); posted = true;
                API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: raw engine post of %u on the sim's emitter 0x%llX -> playing id %u", step, id, (unsigned long long)go, playing); }
        }
    }
    if (!posted && g_cfg.reload_step_via == 5) {
        // THE LISTENER: the player controller's camera manager actor.
        if (auto* pc = API::get()->get_player_controller(0)) {
            if (auto** cm = pc->get_property_data<API::UObject*>(L"PlayerCameraManager")) {
                if (!IsBadReadPtr(cm, sizeof(void*)) && *cm != nullptr) {
                    put(L"Actor", cm, sizeof(void*));
                    cdo->call_function(L"PostEvent", p);
                    if (auto* pr = fn->find_property(L"ReturnValue")) playing = *reinterpret_cast<uint32_t*>(p + pr->get_offset());
                    posted = true; on = *cm;
                }
            }
        }
        if (!posted) API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: no camera manager, falling back", step);
    }
    if (!posted && (g_cfg.reload_step_via == 3 || g_cfg.reload_step_via == 4 || g_cfg.reload_step_via == 5)) {
        // AT THE GUN: AkGameplayStatics.PostEventAtLocation(AkEvent, Location, Orientation, WorldContextObject).
        auto* src = reload_weapon_default_comp();
        have_gun = (src != nullptr) && call_ret_vec3(src, L"K2_GetComponentLocation", &gun);
        auto* lf = cls->find_function(L"PostEventAtLocation");
        static bool s_sig = false;
        if (!s_sig) { s_sig = true; ak_dump_function(L"Function /Script/AkAudio.AkGameplayStatics.PostEventAtLocation"); }
        alignas(16) uint8_t q[256] = {0};
        if (have_gun && lf != nullptr && (size_t)lf->get_properties_size() <= sizeof(q)) {
            bool lok = true;
            auto lput = [&](const wchar_t* name, const void* v, size_t n) { auto* pr = lf->find_property(name); if (pr == nullptr) { lok = false; API::get()->log_info("[Halo-CampE-UEVR] WWISE step: PostEventAtLocation has no '%ls'", name); return; } const int32_t off = pr->get_offset(); if (off >= 0 && (size_t)off + n <= sizeof(q)) memcpy(q + off, v, n); };
            const double loc[3] = {(double)gun.x, (double)gun.y, (double)gun.z};
            const double rot[3] = {0.0, 0.0, 0.0};
            lput(L"AkEvent", &evo, sizeof(void*));
            lput(L"Location", loc, sizeof(loc));
            lput(L"orientation", rot, sizeof(rot));
            lput(L"WorldContextObject", &wpn, sizeof(void*));
            if (lok) {
                cdo->call_function(L"PostEventAtLocation", q);
                if (auto* pr = lf->find_property(L"ReturnValue")) playing = *reinterpret_cast<uint32_t*>(q + pr->get_offset());
            }
        } else if (!have_gun) {
            API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: no gun location, falling back to the pawn", step);
            cdo->call_function(L"PostEvent", p);
            if (auto* pr = fn->find_property(L"ReturnValue")) playing = *reinterpret_cast<uint32_t*>(p + pr->get_offset());
        }
    } else if (!posted && g_cfg.reload_step_via == 1) {
        // The event's own PostOnActor(Actor, PostEventCallback, CallbackMask, bStopWhenAttachedObjectDestroyed).
        auto* pf = evo->get_class() ? evo->get_class()->find_function(L"PostOnActor") : nullptr;
        alignas(16) uint8_t q[256] = {0};
        if (pf != nullptr && (size_t)pf->get_properties_size() <= sizeof(q)) {
            if (auto* pr = pf->find_property(L"Actor")) memcpy(q + pr->get_offset(), &wpn, sizeof(void*));
            evo->call_function(L"PostOnActor", q);
            if (auto* pr = pf->find_property(L"ReturnValue")) playing = *reinterpret_cast<uint32_t*>(q + pr->get_offset());
        }
    } else if (!posted) {
        cdo->call_function(L"PostEvent", p);
        if (auto* pr = fn->find_property(L"ReturnValue")) playing = *reinterpret_cast<uint32_t*>(p + pr->get_offset());
    }
    t_ak_our_post = false;
    if (idp != nullptr) *idp = 0;
    if (g_cfg.reload_vr_log || g_cfg.reload_wwise_dump) {
        float maxdur = -1.0f, atten = -1.0f;
        if (auto* d = evo->get_property_data<float>(L"MaximumDuration")) if (!IsBadReadPtr(d, 4)) maxdur = *d;
        if (auto* a = evo->get_property_data<float>(L"MaxAttenuationRadius")) if (!IsBadReadPtr(a, 4)) atten = *a;
        // Where the post landed: the actor's distance from the camera, the fact that decides audibility.
        Vec3 loc{}; float dist = -1.0f;
        if (have_gun) loc = gun;
        if (have_gun || call_ret_vec3(on, L"K2_GetActorLocation", &loc)) {
            const float dx = loc.x - g_cam_x.load(std::memory_order_relaxed), dy = loc.y - g_cam_y.load(std::memory_order_relaxed), dz = loc.z - g_cam_z.load(std::memory_order_relaxed);
            dist = std::sqrt(dx * dx + dy * dy + dz * dz) / 100.0f;
        }
        API::get()->log_info("[Halo-CampE-UEVR] WWISE step %s: posted '%s' via %d on %ls -> playing id %u (maxdur %.2fs, atten %.0f, %s %.2f m from camera)", step, ev.c_str(), g_cfg.reload_step_via, have_gun ? L"the gun's location" : class_name_of(on).c_str(), playing, maxdur, atten, have_gun ? "gun" : "actor", dist);
    }
}

void audio_dump_begin() {
    if (!g_cfg.reload_audio_dump) return;
    s_ad_at = now_ticks();
    s_ad_phase = 1;
    weapon_components([&](API::UObject* c) {
        if (class_name_of(c).find(L"Audio") != std::wstring::npos) { audio_dump_fields(c); return false; }
        return true;
    });
}
void audio_dump_tick() {
    if (s_ad_phase == 0) return;
    const long long since = now_ticks() - s_ad_at;
    const int want_ms = (s_ad_phase == 1) ? 120 : 400;
    if (since < ms_to_ticks(want_ms)) return;
    auto* arr = API::get()->get_uobject_array();
    auto* wpn = fp_weapon_actor();
    int found = 0, playing = 0;
    if (arr != nullptr) {
        const int32_t nn = arr->get_object_count();
        for (int32_t i = 0; i < nn && playing < 24; ++i) {
            auto* o = static_cast<API::UObject*>(arr->get_object(i));
            if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
            const std::wstring cl = class_name_of(o);
            if (cl.find(L"AudioComponent") == std::wstring::npos) continue;
            ++found;
            bool is_playing = false;
            { alignas(16) uint8_t p[64] = {0}; o->call_function(L"IsPlaying", p); is_playing = p[0] != 0; }
            if (!is_playing) continue;
            ++playing;
            API::UObject* owner = nullptr;
            { alignas(16) uint8_t p[64] = {0}; o->call_function(L"GetOwner", p); owner = *reinterpret_cast<API::UObject**>(p); }
            std::wstring snd = L"-";
            if (auto** ps = o->get_property_data<API::UObject*>(L"Sound"))
                if (!IsBadReadPtr(ps, sizeof(void*)) && *ps != nullptr && !IsBadReadPtr(*ps, sizeof(void*))) snd = (*ps)->get_full_name();
            const auto* fn = o->get_fname();
            API::get()->log_info("[Halo-CampE-UEVR] AUDIODUMP +%dms PLAYING %ls '%ls' owner=%ls%s sound=%ls",
                                 want_ms, cl.c_str(), fn ? fn->to_string().c_str() : L"?",
                                 owner ? class_name_of(owner).c_str() : L"(none)",
                                 (owner != nullptr && owner == wpn) ? " [FP WEAPON]" : "",
                                 snd.c_str());
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] AUDIODUMP +%dms: %d audio components, %d playing", want_ms, found, playing);
    s_ad_phase = (s_ad_phase == 1) ? 2 : 0;
}

// ---- ANIMVARSET (dev): write one named variable on the weapon's anim instance every tick while
// the key is set -- "animvarset=MagnumOverlay_Bool,1". The palette sweep (2026-09-03) showed the
// weapon's parts are UE bones the AnimBP drives, so its variables are the only handle on the
// slide; this is how a candidate is tried in the headset without a build per guess.
void anim_var_set_tick() {
    if (g_cfg.anim_var_set[0] == 0) return;
    static std::string s_last;
    const std::string spec = g_cfg.anim_var_set;
    const size_t comma = spec.find(',');
    if (comma == std::string::npos) return;
    const std::string name(spec.substr(0, comma));
    const double value = atof(spec.c_str() + comma + 1);
    auto* animbp = reload_weapon_anim_instance();
    if (animbp == nullptr) return;
    auto* cls = animbp->get_class();
    if (cls == nullptr) return;
    const std::wstring wname(name.begin(), name.end());
    auto* prop = cls->find_property(wname.c_str());
    if (prop == nullptr) {
        if (spec != s_last) { s_last = spec; API::get()->log_info("[Halo-CampE-UEVR] ANIMVARSET: no property '%s' on %ls", name.c_str(), class_name_of(animbp).c_str()); }
        return;
    }
    const std::wstring pcls = prop->get_class() ? prop->get_class()->get_name() : L"?";
    uint8_t* p = reinterpret_cast<uint8_t*>(animbp) + prop->get_offset();
    if (IsBadWritePtr(p, 8)) return;
    if      (pcls == L"BoolProperty")   static_cast<API::FBoolProperty*>(prop)->set_value_in_object(animbp, value != 0.0);
    else if (pcls == L"ByteProperty" || pcls == L"EnumProperty") *p = (uint8_t)value;
    else if (pcls == L"IntProperty")    *reinterpret_cast<int32_t*>(p) = (int32_t)value;
    else if (pcls == L"FloatProperty")  *reinterpret_cast<float*>(p)   = (float)value;
    else if (pcls == L"DoubleProperty") *reinterpret_cast<double*>(p)  = value;
    if (spec != s_last) {
        s_last = spec;
        API::get()->log_info("[Halo-CampE-UEVR] ANIMVARSET: %s (%ls) = %.3f on %ls, every tick", name.c_str(), pcls.c_str(), value, class_name_of(animbp).c_str());
    }
}

// ---- AMMOSEQ / AMMOSCRUB (dev, 2026-09-03). The weapon AnimBP layers an "ammunition" pose onto
// the gun from the AnimSequence in its FirstPersonPrimaryAmmunition slot, sampled at the frame in
// PrimaryAmmunition_ExplicitFrame (measured: 59 on a loaded pistol, 0 = slide locked back, and
// writing 0 every tick DOES render). That is a per-frame scrub of a sequence the ABP already
// applies to the weapon mesh -- so point the slot at a sequence whose frames carry slide TRAVEL
// (the reloads, the fire) and sweep the frame. ammoseq swaps the slot every tick and restores
// the game's own sequence when cleared; ammoscrub sweeps the frame 0..N-1 at that many frames
// per second, N from the sequence's own frame count (logged once, with its length fields).
TrackedObject s_aq_seq, s_aq_orig, s_aq_inst;
std::string   s_aq_last;
int32_t       s_aq_off = -1;
int           s_aq_frames = 0;
void ammo_seq_tick() {
    auto* animbp = reload_weapon_anim_instance();
    std::string spec = g_cfg.ammo_seq;
    // String values arrive with the file's line end still attached (2026-09-03: the name carried
    // a stray CR and matched nothing, and the miss retried a full object walk EVERY tick).
    while (!spec.empty() && (spec.back() == '\r' || spec.back() == '\n' || spec.back() == ' ' || spec.back() == '\t')) spec.pop_back();
    if (spec == "0" || _stricmp(spec.c_str(), "off") == 0) spec.clear();
    if (spec.empty()) {
        if (s_aq_off >= 0) {
            if (auto* inst = s_aq_inst.get()) {
                auto** pp = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(inst) + s_aq_off);
                if (!IsBadWritePtr(pp, sizeof(void*))) *pp = s_aq_orig.get();
            }
            API::get()->log_info("[Halo-CampE-UEVR] AMMOSEQ: slot restored to the game's sequence");
            s_aq_off = -1; s_aq_last.clear(); s_aq_frames = 0;
            s_aq_seq = TrackedObject{}; s_aq_orig = TrackedObject{}; s_aq_inst = TrackedObject{};
        }
    } else if (animbp != nullptr && (spec != s_aq_last || s_aq_inst.get() != animbp)) {
        s_aq_last = spec;
        s_aq_off = -1;
        s_aq_inst.set(animbp);   // a miss below is final for this spec on this instance: no per-tick retry
        // The slot: the ObjectProperty on the instance's class chain whose name ends in
        // FirstPersonPrimaryAmmunition (the dump prints it as Animations/FirstPersonPrimaryAmmunition).
        int32_t off = -1;
        for (API::UStruct* st = animbp->get_class(); st != nullptr && off < 0; st = st->get_super_struct()) {
            if (IsBadReadPtr(st, sizeof(void*))) break;
            for (API::FField* f = st->get_child_properties(); f != nullptr; f = f->get_next()) {
                if (IsBadReadPtr(f, sizeof(void*))) break;
                auto* fc = f->get_class();
                if (fc == nullptr || fc->get_name() != L"ObjectProperty") continue;
                const auto* fn = f->get_fname();
                if (fn == nullptr) continue;
                const std::wstring nm = fn->to_string();
                static const std::wstring want = L"FirstPersonPrimaryAmmunition";
                if (nm.size() >= want.size() && nm.compare(nm.size() - want.size(), want.size(), want) == 0) {
                    off = static_cast<API::FProperty*>(f)->get_offset(); break;
                }
            }
        }
        if (off < 0) { API::get()->log_info("[Halo-CampE-UEVR] AMMOSEQ: no FirstPersonPrimaryAmmunition slot on %ls", class_name_of(animbp).c_str()); return; }
        auto* seq = find_anim_sequence(spec);
        if (seq == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] AMMOSEQ: no AnimSequence matching '%s'", spec.c_str()); return; }
        auto** pp = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(animbp) + off);
        if (IsBadWritePtr(pp, sizeof(void*))) return;
        s_aq_off = off; s_aq_inst.set(animbp); s_aq_orig.set(*pp); s_aq_seq.set(seq);
        // The sequence's own length fields, and the frame count the sweep uses.
        float len = 0.0f; int frames = 0, keys = 0;
        for (API::UStruct* st = seq->get_class(); st != nullptr; st = st->get_super_struct()) {
            if (IsBadReadPtr(st, sizeof(void*))) break;
            for (API::FField* f = st->get_child_properties(); f != nullptr; f = f->get_next()) {
                if (IsBadReadPtr(f, sizeof(void*))) break;
                auto* fc = f->get_class();
                const auto* fn = f->get_fname();
                if (fc == nullptr || fn == nullptr) continue;
                const std::wstring cls = fc->get_name();
                const std::wstring nm  = fn->to_string();
                const int32_t o = static_cast<API::FProperty*>(f)->get_offset();
                const uint8_t* q = reinterpret_cast<const uint8_t*>(seq) + o;
                if (IsBadReadPtr(q, 8)) continue;
                double v = 0.0; bool num = true;
                if      (cls == L"FloatProperty")  v = *reinterpret_cast<const float*>(q);
                else if (cls == L"DoubleProperty") v = *reinterpret_cast<const double*>(q);
                else if (cls == L"IntProperty")    v = *reinterpret_cast<const int32_t*>(q);
                else num = false;
                if (!num) continue;
                if (nm.find(L"Length") != std::wstring::npos || nm.find(L"Frame") != std::wstring::npos ||
                    nm.find(L"Key") != std::wstring::npos || nm.find(L"Rate") != std::wstring::npos) {
                    API::get()->log_info("[Halo-CampE-UEVR] AMMOSEQ   %ls (%ls@0x%X) = %.3f", nm.c_str(), cls.c_str(), (unsigned)o, v);
                }
                if (nm == L"SequenceLength") len = (float)v;
                if (nm == L"NumberOfSampledFrames" || nm == L"NumFrames") frames = (int)v;
                if (nm == L"NumberOfSampledKeys" || nm == L"NumberOfKeys") keys = (int)v;
            }
        }
        s_aq_frames = frames > 1 ? frames : (keys > 1 ? keys : (len > 0.0f ? (int)(len * 30.0f + 0.5f) : 60));
        API::get()->log_info("[Halo-CampE-UEVR] AMMOSEQ: slot @0x%X %ls -> %ls, sweep uses %d frames (len %.3f s)",
                             (unsigned)off, s_aq_orig.get() ? s_aq_orig.get()->get_full_name().c_str() : L"null",
                             seq->get_full_name().c_str(), s_aq_frames, len);
    }
    if (!spec.empty() && s_aq_off >= 0 && animbp != nullptr && s_aq_inst.get() == animbp) {
        auto** pp = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(animbp) + s_aq_off);
        if (!IsBadWritePtr(pp, sizeof(void*)) && s_aq_seq.get() != nullptr) *pp = s_aq_seq.get();
    }
    if (g_cfg.ammo_scrub > 0.0f && animbp != nullptr) {
        auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame");
        if (p != nullptr && !IsBadWritePtr(p, sizeof(int32_t))) {
            static long long s_t0 = 0;
            if (s_t0 == 0) s_t0 = now_ticks();
            const double sec = (double)(now_ticks() - s_t0) / (double)ms_to_ticks(1000);
            const int n = s_aq_frames > 1 ? s_aq_frames : 60;
            const int frame = (int)(sec * (double)g_cfg.ammo_scrub) % n;
            *p = frame;
            static long long s_said = 0;
            if (now_ticks() - s_said > ms_to_ticks(500)) {
                s_said = now_ticks();
                API::get()->log_info("[Halo-CampE-UEVR] AMMOSCRUB: frame %d of %d", frame, n);
            }
        }
    }
}
