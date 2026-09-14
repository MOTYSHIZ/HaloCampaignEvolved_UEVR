// vehcam (fork feature, Experimental): the driver body hide while mounted and the Warthog hull resolve the rigid seat camera reads.
// Textual fragment, included by Arms.cpp at namespace halo scope, at the end of the file. Moved verbatim; not compiled on its own.
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
