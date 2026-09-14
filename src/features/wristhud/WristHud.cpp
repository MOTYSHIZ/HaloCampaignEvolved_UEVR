#include "WristHud.hpp"

#include "BlamDrive.hpp"          // the unit state the radar scan is handed
#include "Config.hpp"
#include "Holster.hpp"
#include "Markers.hpp"            // holster_marker_place_rot / scale / room_to_world
#include "core/MarkerFaces.hpp"         // holster_world_to_room
#include "Math.hpp"
#include "Rig.hpp"              // call_ret_vec3
#include "MotionAimControl.hpp"   // get_pose, g_stick_mode_active
#include "Reticule.hpp"           // widget_quad_begin/finish -- THE one copy of the quad recipe
#include "ArmDriver.hpp"          // palette_weapon_mode: hudwpnanchor 3
#include "UeObject.hpp"
#include "core/Clock.hpp"               // clock::now_ms: the radar's wall-clock cadences
#include "core/HiddenReload.hpp"        // g_wristhud_hide_cradle
#include "core/host/BlamDriveState.hpp" // read_ptr, g_tls_index: the radar's object-table walk
#include "core/host/HolsterState.hpp"   // the holster's grenade meshes, as blip art
#include "core/fixes/TickStage.hpp"
#include "uevr/API.hpp"

#include <Windows.h>
#include <Xinput.h>
#include <intrin.h>
#include <cmath>
#include <cstring>
#include <string>
#include <functional>
#include <vector>

using uevr::API;

namespace halo {

extern std::atomic<float> g_view_base_yaw;   // Plugin.cpp: the yaw handed to UEVR every frame
// The palette weapon's world pose as last resolved (published in the palette weapon's Plugin.cpp block),
// read only by hudwpnanchor 3 while armdriver 3 owns the weapon.
extern std::atomic<float> g_dbg_pose_w_x, g_dbg_pose_w_y, g_dbg_pose_w_z, g_dbg_pose_w_w;

std::atomic<bool> g_wristhud_lt{false};

// The author's widget colour chain (Reticule.cpp): the SlateUI bind and the tint. Both run per frame
// on every hosted panel. A panel's gain multiplier reaches the tint through the widget_tint_mul hook,
// answered from here for the call in progress on this thread.
bool bind_widget_slate_ui(API::UObject* comp);
void apply_widget_tint(API::UObject* comp, bool force);
namespace {
thread_local float tl_tint_mul = 1.0f;
void wristhud_apply_widget_tint(API::UObject* comp, float mul, bool force) {
    tl_tint_mul = mul;
    apply_widget_tint(comp, force);
    tl_tint_mul = 1.0f;
}
} // namespace

// ---- THE CRADLE'S NUMBER (the hidden reload, 2026-09-07): the host's refill must not show on
// the wrist before the mag is in. The cradle is only a display, so its number is written to 0
// every frame while the hidden reload is pending. The number field is found once per hosted
// widget: the first TextBlock whose text is all digits (wristhudammotext names it outright), and
// the tree is logged once so the name can be pinned.
std::wstring wh_text_of(API::UObject* w) {
    std::wstring out;
    auto* cls = w->get_class(); if (cls == nullptr || cls->find_function(L"GetText") == nullptr) return out;
    auto* ktl = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetTextLibrary");
    auto* fn = ktl ? ktl->find_function(L"Conv_TextToString") : nullptr;
    auto* cdo = ktl ? ktl->get_class_default_object() : nullptr;
    if (fn == nullptr || cdo == nullptr) return out;
    alignas(16) uint8_t t[64] = {0};
    w->call_function(L"GetText", t);                 // FText ReturnValue at 0 (the only param)
    auto* pin = fn->find_property(L"InText"); auto* pret = fn->find_property(L"ReturnValue");
    if (pin == nullptr || pret == nullptr) return out;
    alignas(16) uint8_t q[96] = {0};
    memcpy(q + pin->get_offset(), t, 24);
    cdo->call_function(L"Conv_TextToString", q);
    struct FStr { const wchar_t* data; int32_t num; int32_t max; };
    const auto* fs = reinterpret_cast<const FStr*>(q + pret->get_offset());
    if (fs->data != nullptr && fs->num > 1 && fs->num < 256 && !IsBadReadPtr(fs->data, (size_t)fs->num * 2)) out.assign(fs->data, (size_t)fs->num - 1);
    return out;
}
void wh_walk(API::UObject* w, int depth, const std::function<bool(API::UObject*, int)>& f) {
    if (w == nullptr || depth > 24 || IsBadReadPtr(w, sizeof(void*))) return;
    if (!f(w, depth)) return;
    if (auto** wt = w->get_property_data<API::UObject*>(L"WidgetTree")) if (!IsBadReadPtr(wt, sizeof(void*)) && *wt != nullptr)
        if (auto** root = (*wt)->get_property_data<API::UObject*>(L"RootWidget")) if (!IsBadReadPtr(root, sizeof(void*))) wh_walk(*root, depth + 1, f);
    auto* cls = w->get_class();
    if (cls != nullptr && cls->find_function(L"GetChildrenCount") != nullptr) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        w->call_function(L"GetChildrenCount", p);
        const int32_t cn = *reinterpret_cast<int32_t*>(p);
        for (int32_t i = 0; i < cn && i < 32; ++i) {
            alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
            *reinterpret_cast<int32_t*>(q) = i;
            w->call_function(L"GetChildAt", q);
            wh_walk(*reinterpret_cast<API::UObject**>(q + 8), depth + 1, f);
        }
    }
}
void wh_cradle_dump(API::UObject* widget) {
    static bool s_done = false; if (s_done || widget == nullptr || g_cfg.palette_weapon_log == 0) return; s_done = true;
    int lines = 0;
    wh_walk(widget, 0, [&](API::UObject* w, int depth) {
        if (++lines > 80) return false;
        const std::wstring cls = class_name_of(w);
        const auto* fn = w->get_fname();
        const std::wstring txt = cls.find(L"TextBlock") != std::wstring::npos ? wh_text_of(w) : std::wstring();
        API::get()->log_info("[Halo-CampE-UEVR] WRISTHUD cradle tree %*s%ls (%ls)%ls%ls%ls", depth * 2, "", fn ? fn->to_string().c_str() : L"?", cls.c_str(), txt.empty() ? L"" : L" text='", txt.c_str(), txt.empty() ? L"" : L"'");
        return true;
    });
}
// The ammo box (StackBox_Ammo): its digit children are added by the game after hosting, so the
// search repeats every 2 s until something is found, and the box's subtree is logged once when
// it has children. With no text field the digits are collapsed while the reload is hidden.
TrackedObject s_ammo_box; bool s_ammo_collapsed = false; uint8_t s_ammo_vis_orig = 0;
void wh_cradle_ammo_dump() {
    static bool s_done = false; if (s_done) return;
    auto* box = s_ammo_box.get(); if (box == nullptr) return;
    int n = 0;
    wh_walk(box, 0, [&](API::UObject* w, int depth) {
        if (++n > 40) return false;
        const std::wstring cls = class_name_of(w);
        const auto* fn = w->get_fname();
        const std::wstring txt = cls.find(L"Text") != std::wstring::npos ? wh_text_of(w) : std::wstring();
        API::get()->log_info("[Halo-CampE-UEVR] WRISTHUD ammo box %*s%ls (%ls)%ls%ls%ls", depth * 2, "", fn ? fn->to_string().c_str() : L"?", cls.c_str(), txt.empty() ? L"" : L" text='", txt.c_str(), txt.empty() ? L"" : L"'");
        return true;
    });
    if (n > 1) { s_done = true; API::get()->log_info("[Halo-CampE-UEVR] WRISTHUD ammo box: %d widget(s) listed", n); }
}
void wh_cradle_restore() {
    if (!s_ammo_collapsed) return;
    s_ammo_collapsed = false;
    if (auto* box = s_ammo_box.get()) { alignas(16) uint8_t p[64] = {0}; p[0] = s_ammo_vis_orig; box->call_function(L"SetVisibility", p); }
}
API::UObject* wh_cradle_number(API::UObject* widget) {
    static API::UObject* s_widget = nullptr; static TrackedObject s_num; static ULONGLONG s_last = 0;
    const ULONGLONG now_ms = GetTickCount64();
    if (widget == s_widget) { if (auto* o = s_num.get()) return o; if (now_ms - s_last < 2000) return nullptr; }
    s_last = now_ms;
    if (widget != s_widget) { s_ammo_box = TrackedObject{}; s_ammo_collapsed = false; }
    s_widget = widget; s_num = TrackedObject{};
    std::string want(g_cfg.wrist_hud_ammo_text);
    while (!want.empty() && (unsigned char)want.back() <= ' ') want.pop_back();
    while (!want.empty() && (unsigned char)want.front() <= ' ') want.erase(want.begin());
    const std::wstring wwant(want.begin(), want.end());
    API::UObject* found = nullptr;
    wh_walk(widget, 0, [&](API::UObject* w, int) {
        if (found != nullptr) return false;
        const auto* fn = w->get_fname();
        if (fn != nullptr && s_ammo_box.get() == nullptr && fn->to_string() == L"StackBox_Ammo") s_ammo_box.set(w);
        if (class_name_of(w).find(L"Text") == std::wstring::npos) return true;
        if (!wwant.empty()) { if (fn != nullptr && fn->to_string() == wwant) found = w; return true; }
        const std::wstring txt = wh_text_of(w);
        if (!txt.empty() && txt.find_first_not_of(L"0123456789") == std::wstring::npos) found = w;
        return true;
    });
    if (found != nullptr) { s_num.set(found); API::get()->log_info("[Halo-CampE-UEVR] WRISTHUD cradle number field: %ls", found->get_fname() ? found->get_fname()->to_string().c_str() : L"?"); }
    else { static int s_said = 0; if (s_said++ < 3) API::get()->log_info("[Halo-CampE-UEVR] WRISTHUD cradle number field: NOT FOUND (ammo box %s)", s_ammo_box.get() ? "found" : "missing"); }
    return s_num.get();
}
void wh_cradle_write_zero(API::UObject* widget) {
    if (widget == nullptr) return;
    auto* num = wh_cradle_number(widget);
    if (num == nullptr) { wh_cradle_ammo_dump(); return; }   // nothing hidden, ever: the number is written or left alone
    // The FText "0", made once through Conv_IntToText and kept (never destroyed, one small leak).
    static alignas(16) uint8_t s_zero[32] = {0}; static bool s_zero_ok = false;
    if (!s_zero_ok) {
        auto* ktl = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetTextLibrary");
        auto* fn = ktl ? ktl->find_function(L"Conv_IntToText") : nullptr;
        auto* cdo = ktl ? ktl->get_class_default_object() : nullptr;
        if (fn == nullptr || cdo == nullptr) return;
        auto* pv = fn->find_property(L"Value"); auto* pmin = fn->find_property(L"MinimumIntegralDigits"); auto* pmax = fn->find_property(L"MaximumIntegralDigits"); auto* pret = fn->find_property(L"ReturnValue");
        if (pv == nullptr || pret == nullptr) return;
        alignas(16) uint8_t q[128] = {0};
        *reinterpret_cast<int32_t*>(q + pv->get_offset()) = 0;
        if (pmin) *reinterpret_cast<int32_t*>(q + pmin->get_offset()) = 1;
        if (pmax) *reinterpret_cast<int32_t*>(q + pmax->get_offset()) = 324;
        cdo->call_function(L"Conv_IntToText", q);
        memcpy(s_zero, q + pret->get_offset(), 24);
        s_zero_ok = true;
    }
    auto* cls = num->get_class(); auto* sfn = cls ? cls->find_function(L"SetText") : nullptr; if (sfn == nullptr) return;
    auto* pin = sfn->find_property(L"InText"); if (pin == nullptr) return;
    alignas(16) uint8_t p[64] = {0};
    memcpy(p + pin->get_offset(), s_zero, 24);
    num->call_function(L"SetText", p);
}

API::UObject* load_asset_by_path(const char* path);   // Reticule.cpp

namespace {

constexpr int MAX_SLOTS = 6;

struct WhSlot {
    std::wstring  match;    // class-name substring from wristhudclasses / wristhudclassesr
    TrackedObject comp;     // our WidgetComponent
    TrackedObject widget;   // the game's widget instance we host
    // MATURITY: the candidate seen last sweep. Hosting happens only when the SAME instance shows
    // up in two consecutive sweeps (~4 s apart) -- a level transition rebuilds the HUD, and an
    // instance grabbed mid-construction hosts as a BLANK panel (measured: healthy tint chain,
    // bound target, black quad -- the widget itself had nothing in it yet).
    uevr::API::UObject* cand = nullptr;
    bool          right = false;   // anchored to the aim hand instead of the off hand
    bool          failed = false;
    int           misses = 0;      // sweeps that found nothing for this slot; eight = back off to one per 1200 ticks
};
WhSlot s_slots[MAX_SLOTS];
int    s_slot_count = 0;
std::string s_classes_active;   // both cfg strings, concatenated, as last parsed

uint32_t s_tick = 0;
uint32_t s_scan_tick = 0;

// Census bookkeeping: each distinct live widget class logs once per session. The census keeps
// the sweep alive only for its first ~25 passes (~100 s of play) -- long enough for every HUD
// element to exist and be named, without a standing full-array walk for the rest of the session.
std::vector<std::wstring> s_census_seen;
int s_census_lines = 0;
int s_census_sweeps = 0;

// Same test the reticule uses (its copy is internal): a CONSTRUCTED widget lives under
// /Engine/Transient; the class archetype never does, and binding or removing the archetype is
// the documented way to delete a HUD element for the whole session.
bool live_widget_instance(API::UObject* o) {
    for (API::UObject* p = o; p != nullptr; p = p->get_outer()) {
        const auto* fn = p->get_fname();
        if (fn != nullptr && fn->to_string().find(L"Transient") != std::wstring::npos) return true;
    }
    return false;
}

// Take the widget off the flat HUD and give it to our quad -- the reticule's host sequence.
void wh_host(API::UObject* comp, API::UObject* w) {
    { alignas(16) uint8_t p[64] = {0}; p[0] = 0; w->call_function(L"SetVisibility", p); }  // SelfHitTestInvisible=0? 0 = Visible
    { alignas(16) uint8_t p[64] = {0}; w->call_function(L"RemoveFromParent", p); }
    { alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<void**>(p) = w;
      comp->call_function(L"SetWidget", p); }
}

void parse_one_list(const char* p, bool right) {
    while (*p != 0 && s_slot_count < MAX_SLOTS) {
        const char* e = strchr(p, ',');
        size_t n = (e != nullptr) ? (size_t)(e - p) : strlen(p);
        // Trim whitespace and stray CR from the token: a cfg edited by different tools carries
        // different line endings, and an invisible trailing byte makes exactly the LAST class in
        // the list silently never match -- the worst kind of bug to stare at.
        while (n > 0 && (p[n - 1] == '\r' || p[n - 1] == '\n' || p[n - 1] == ' ' || p[n - 1] == '\t')) --n;
        size_t s0 = 0;
        while (s0 < n && (p[s0] == ' ' || p[s0] == '\t')) ++s0;
        if (n > s0) {
            std::string a(p + s0, n - s0);
            s_slots[s_slot_count].match = std::wstring(a.begin(), a.end());
            s_slots[s_slot_count].right = right;
            ++s_slot_count;
        }
        p += (e != nullptr) ? (size_t)(e - p) : strlen(p);
        if (*p == ',') ++p;
    }
}

void parse_slots() {
    // The blend mode is baked into a quad at construction, so it rides the change detector: a
    // wristhudblend edit reads as a "new class list" and rebuilds the panels in place.
    std::string both = std::string(g_cfg.wrist_hud_classes) + "|" + g_cfg.wrist_hud_classes_r
                     + "|" + std::to_string(g_cfg.wrist_hud_blend)
                     + "|" + std::to_string(g_cfg.wrist_hud_blend_r);
    if (s_classes_active == both) return;
    s_classes_active = both;
    // A re-parse must not ORPHAN live panels: hide each component before dropping its handle, or
    // the quads freeze mid-world holding their hosted widgets. (The widget itself stays hosted --
    // giving it back to the flat HUD is a bigger job; a level load restores it.)
    for (int i = 0; i < s_slot_count; ++i)
        if (auto* c = s_slots[i].comp.get()) holster_marker_show(c, false);
    for (int i = 0; i < MAX_SLOTS; ++i) s_slots[i] = WhSlot{};
    s_slot_count = 0;
    parse_one_list(g_cfg.wrist_hud_classes, false);
    parse_one_list(g_cfg.wrist_hud_classes_r, true);
    // The parse, as parsed -- lengths included, because an invisible byte in a match string is
    // indistinguishable from a healthy one in every other log line.
    for (int i = 0; i < s_slot_count; ++i)
        API::get()->log_info("[Halo-CampE-UEVR] WRISTHUD slot %d: '%ls' (len %d, %s wrist)",
                             i, s_slots[i].match.c_str(), (int)s_slots[i].match.size(),
                             s_slots[i].right ? "right" : "left");
}

// One paced object-array sweep serves both the census and the slot binding. The walk is the
// expensive full-array kind, so it runs only while it has a consumer: an unbound slot, or the
// census still collecting.
void sweep(API::UObject* owner) {
    const bool census_live = s_census_lines < 60 && s_census_sweeps < 25;
    bool want = census_live;
    for (int i = 0; i < s_slot_count; ++i)
        if (!s_slots[i].failed && s_slots[i].widget.get() == nullptr && (s_slots[i].misses < 8 || s_tick - s_scan_tick >= 1200)) want = true;
    if (!want) return;
    if (s_tick - s_scan_tick < 120) return;
    s_scan_tick = s_tick;
    // A miss counts only once some slot is bound, i.e. the HUD exists: in the menu and the load
    // every sweep finds nothing, and counting those backed the mission's first bind off to one
    // look per 37 s (the panels came up 44 s after the gun, 2026-09-07).
    bool any_bound = false;
    for (int i = 0; i < s_slot_count; ++i) if (s_slots[i].widget.get() != nullptr) { any_bound = true; break; }
    if (any_bound) for (int i = 0; i < s_slot_count; ++i)
        if (!s_slots[i].failed && s_slots[i].widget.get() == nullptr) ++s_slots[i].misses;
    if (census_live) ++s_census_sweeps;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring cls = class_name_of(o);
        if (cls.find(L"WBP_") == std::wstring::npos) continue;
        if (!live_widget_instance(o)) {
            // DIAGNOSTIC: an instance matching an UNBOUND slot that fails the transient test is
            // exactly the motion-tracker mystery -- print its outer chain so the liveness test
            // can be corrected from evidence instead of theory.
            static int s_diag = 0;
            if (s_diag < 8) {
                for (int si = 0; si < s_slot_count; ++si) {
                    if (s_slots[si].failed || s_slots[si].widget.get() != nullptr) continue;
                    if (cls.find(s_slots[si].match) == std::wstring::npos) continue;
                    ++s_diag;
                    std::wstring chain;
                    for (API::UObject* pp = o; pp != nullptr; pp = pp->get_outer()) {
                        const auto* fn = pp->get_fname();
                        if (fn != nullptr) { chain += fn->to_string(); chain += L" < "; }
                        if (chain.size() > 180) break;
                    }
                    API::get()->log_info("[Halo-CampE-UEVR] WRISTHUD non-live match %ls: outers %ls",
                                         cls.c_str(), chain.c_str());
                    break;
                }
            }
            continue;
        }

        // CENSUS: one line per distinct class, capped. This is the survey the hosting cfg is
        // filled from -- everything the HUD has live, by its real name.
        if (s_census_lines < 60) {
            bool seen = false;
            for (const auto& s : s_census_seen) if (s == cls) { seen = true; break; }
            if (!seen) {
                s_census_seen.push_back(cls);
                ++s_census_lines;
                API::get()->log_info("[Halo-CampE-UEVR] WRISTHUD census: %ls", cls.c_str());
            }
        }

        // SLOT BINDING: first live instance whose class contains the slot's substring, hosted
        // only on its SECOND consecutive sighting (see the maturity note on WhSlot).
        for (int si = 0; si < s_slot_count; ++si) {
            WhSlot& sl = s_slots[si];
            if (sl.failed || sl.widget.get() != nullptr) continue;
            if (cls.find(sl.match) == std::wstring::npos) continue;

            if (sl.cand != o) { sl.cand = o; continue; }

            if (sl.comp.get() == nullptr) {
                // Build the quad NOW, with the widget in hand: registration must happen after
                // SetWidget or the render target is a degenerate (0,0) quad (see Reticule.hpp).
                bool compensated = false;
                auto* comp = widget_quad_begin(owner, sl.right ? g_cfg.wrist_hud_blend_r
                                                              : g_cfg.wrist_hud_blend, &compensated);
                if (comp == nullptr) { sl.failed = true; continue; }
                { alignas(16) uint8_t p[64] = {0};
                  auto* d = reinterpret_cast<double*>(p);
                  d[0] = g_cfg.wrist_hud_draw; d[1] = g_cfg.wrist_hud_draw;
                  comp->call_function(L"SetDrawSize", p); }
                wh_host(comp, o);
                widget_quad_finish(owner, comp, 10.0f);
                sl.comp.set(comp);
                if (sl.match.find(L"WeaponCradle") != std::wstring::npos) wh_cradle_dump(o);
            } else {
                // Component survived a HUD rebuild; only the widget died. Re-host the new one.
                wh_host(sl.comp.get(), o);
            }
            sl.widget.set_at(o, i); sl.misses = 0;
            API::get()->log_info("[Halo-CampE-UEVR] WRISTHUD hosting %ls (slot %d, '%ls')",
                                 cls.c_str(), si, sl.match.c_str());
        }
    }
}

// NATIVE-BLIP INVESTIGATION (doctrine at Config::tracker_dump). One-shot on value change, so a
// capture can be fired by a cfg edit while the player is mid-firefight. Two sweeps over the object
// array:
//   A. every UObject whose OUTER chain reaches the live tracker instance -- the widget tree the
//      tracker owns outright;
//   B. every live widget ANYWHERE whose class contains "Blip"/"blip", with a short outer chain --
//      because CreateWidget parents new widgets to the world, not to the panel that displays
//      them, so a blip can be fed to the tracker without ever appearing in sweep A.
// The verdict is the DIFFERENCE between a native capture and a hosted capture, not either alone.
void tracker_dump_probe() {
    static int s_armed_as = 0;
    if (g_cfg.tracker_dump == s_armed_as) return;
    s_armed_as = g_cfg.tracker_dump;
    if (s_armed_as == 0) return;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count();

    API::UObject* tracker = nullptr;
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o).find(L"WBP_MotionTracker") == std::wstring::npos) continue;
        if (!live_widget_instance(o)) continue;
        tracker = o;
        break;
    }
    const auto* tfn = (tracker != nullptr) ? tracker->get_fname() : nullptr;
    API::get()->log_info("[Halo-CampE-UEVR] TRACKERDUMP %d: tracker %ls @%p",
                         s_armed_as,
                         (tfn != nullptr) ? tfn->to_string().c_str() : L"NOT FOUND",
                         (void*)tracker);

    int total_a = 0, lines_a = 0;
    int total_b = 0, lines_b = 0;
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || o == tracker || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring cls = class_name_of(o);

        if (tracker != nullptr) {
            int depth = 0;
            for (API::UObject* p = o->get_outer(); p != nullptr && depth < 12; p = p->get_outer(), ++depth) {
                if (p != tracker) continue;
                ++total_a;
                if (lines_a < 48) {
                    ++lines_a;
                    const auto* fn = o->get_fname();
                    API::get()->log_info("[Halo-CampE-UEVR] TRACKERDUMP A: %ls '%ls'",
                                         cls.c_str(),
                                         (fn != nullptr) ? fn->to_string().c_str() : L"?");
                }
                break;
            }
        }

        if (cls.find(L"Blip") == std::wstring::npos && cls.find(L"blip") == std::wstring::npos) continue;
        if (!live_widget_instance(o)) continue;
        ++total_b;
        if (lines_b < 24) {
            ++lines_b;
            std::wstring chain;
            int depth = 0;
            for (API::UObject* p = o->get_outer(); p != nullptr && depth < 6; p = p->get_outer(), ++depth) {
                const auto* fn = p->get_fname();
                if (fn != nullptr) { chain += fn->to_string(); chain += L" < "; }
                if (chain.size() > 160) break;
            }
            API::get()->log_info("[Halo-CampE-UEVR] TRACKERDUMP B: %ls outers %ls",
                                 cls.c_str(), chain.c_str());
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] TRACKERDUMP %d done: %d under tracker, %d blip-class live",
                         s_armed_as, total_a, total_b);
}

// ---- COLOURED RADAR DOTS ----------------------------------------------------------------------
// Red = covenant, blue = human, by request -- the grenade meshes were a placeholder and read as
// clutter at radar size. Recipe is the mesh reticule's proven colour chain: an engine sphere, a
// MID from Widget3DPassThrough_Opaque, and a solid-colour render target as its SlateUI, tinted up
// by wristradargain against the same pre-exposure crush the panels fight with aimwidgetgain.
// One render target per BODY CLASS (+0x177), built on demand and cached for the session. An
// unsurveyed class takes blip_color_other and logs itself once, so the value set gets filled in
// from play. NOTE this is species, not faction: an Elite and a marine share 0x0E.
// ---- SPECIES NAMING ------------------------------------------------------------------------
// A tag id is per MAP, so a palette keyed on it dies at every level load and lands on some
// unrelated species. The UNREAL ACTOR standing at the same spot carries a class name that does
// not change between levels, so that is the durable key. Once per newly seen tag id, sweep the
// actor array for the nearest pawn-ish actor to that contact's world position and remember the
// name. One sweep per species, never per frame.
struct SpeciesName {
    uint32_t     cls = 0;
    std::wstring name;
    bool         tried = false;
};
SpeciesName s_species[24];
int          s_species_n = 0;

const std::wstring* species_name(uint32_t cls) {
    for (int i = 0; i < s_species_n; ++i)
        if (s_species[i].cls == cls) return s_species[i].name.empty() ? nullptr : &s_species[i].name;
    return nullptr;
}

// Blam world units -> UE world cm. Same fit the grenade and vehicle work uses: scale by 304.8
// with Y negated.
Vec3 blam_to_ue(float bx, float by, float bz) {
    return Vec3{bx * 304.8f, -by * 304.8f, bz * 304.8f};
}

void species_resolve(uint32_t cls, const Vec3& ue) {
    if (cls == 0) return;
    for (int i = 0; i < s_species_n; ++i) if (s_species[i].cls == cls) return;
    if (s_species_n >= 24) return;
    SpeciesName& e = s_species[s_species_n++];
    e.cls = cls;
    e.tried = true;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count();
    float best = 250.0f * 250.0f;   // 2.5 m: closer than any two bipeds stand to each other
    std::wstring best_name;
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring c = class_name_of(o);
        // BIPEDS ONLY. The convention is BP_<Species>BipedActor_C (confirmed in the field:
        // BP_GruntBipedActor_C). A looser filter grabbed whatever stood nearest and produced
        // nonsense names -- a decal and an equipment actor were the first two matches.
        if (c.find(L"Biped") == std::wstring::npos) continue;
        Vec3 loc{};
        if (!call_ret_vec3(o, L"K2_GetActorLocation", &loc)) continue;
        const float dx = loc.x - ue.x, dy = loc.y - ue.y, dz = loc.z - ue.z;
        const float d2 = dx * dx + dy * dy + dz * dz;
        if (d2 >= best) continue;
        best = d2;
        best_name = c;
    }
    e.name = best_name;
    if (!best_name.empty())
        API::get()->log_info("[Halo-CampE-UEVR] RADAR species 0x%08X = %ls (%.1f m away) "
                             "-- colour with blipname=<substring>,r,g,b",
                             cls, best_name.c_str(), std::sqrt(best) / 100.0f);
    else
        API::get()->log_info("[Halo-CampE-UEVR] RADAR species 0x%08X = no actor within 2.5 m", cls);
}

// ONE white render target, shared. The colour used to live in the texture, which is RGBA8: a
// channel is either 0 or at least 1/255, and 1/255 times the brightness gain is ~6, far past
// clipping. So every colour arrived fully saturated and the tonemapper desaturated the brightest
// toward white -- "grunts look orange", "coop looks like a white dot". Colour and brightness now
// both live in the TINT, which is a float vector, so exact hues and genuinely dark shades work.
API::UObject* dot_rt() {
    static TrackedObject s_white;
    if (auto* live = s_white.get()) return live;
    auto* rt = make_color_rt(1.0f, 1.0f, 1.0f, 1.0f, 16);
    if (rt != nullptr) s_white.set(rt);
    return rt;
}

// The palette lookup: an explicit blipcolor entry, else a hue derived from the species id. Values
// are ordinary 0..1 -- 0,0,1 is full blue, 0,0,0.35 is a dark blue.
void dot_color_for(uint32_t cls, float* out) {
    // NAME first -- it survives level loads, where a tag id does not.
    if (const std::wstring* nm = species_name(cls)) {
        for (int i = 0; i < g_cfg.blip_name_n; ++i) {
            const std::string& m = g_cfg.blip_name[i].match;
            if (m.empty()) continue;
            const std::wstring w(m.begin(), m.end());
            if (nm->find(w) == std::wstring::npos) continue;
            out[0] = g_cfg.blip_name[i].r;
            out[1] = g_cfg.blip_name[i].g;
            out[2] = g_cfg.blip_name[i].b;
            return;
        }
    }
    for (int i = 0; i < g_cfg.blip_color_n; ++i) {
        if (g_cfg.blip_color[i].cls != cls) continue;
        out[0] = g_cfg.blip_color[i].r;
        out[1] = g_cfg.blip_color[i].g;
        out[2] = g_cfg.blip_color[i].b;
        return;
    }
    if (cls == 0) { out[0] = g_cfg.blip_color_other[0]; out[1] = g_cfg.blip_color_other[1];
                    out[2] = g_cfg.blip_color_other[2]; return; }
    // Derived: distinct hue per species, deterministic, so one map always paints the same.
    uint32_t h = cls * 2654435761u;
    h ^= h >> 15;
    const float hue = (float)(h % 360u);
    const float sec = hue / 60.0f;
    const float x = 1.0f - std::fabs(std::fmod(sec, 2.0f) - 1.0f);
    const int   k = (int)sec % 6;
    const float r[6] = {1, x, 0, 0, x, 1};
    const float g[6] = {x, 1, 1, x, 0, 0};
    const float b[6] = {0, 0, x, 1, 1, x};
    out[0] = r[k]; out[1] = g[k]; out[2] = b[k];
    static uint32_t s_seen[32]; static int s_nseen = 0;
    for (int i = 0; i < s_nseen; ++i) if (s_seen[i] == cls) return;
    if (s_nseen < 32) s_seen[s_nseen++] = cls;
    API::get()->log_info("[Halo-CampE-UEVR] RADAR: species 0x%08X -> derived hue %.0f "
                         "(override with blipcolor=%08X,r,g,b)", cls, hue, cls);
}

void dot_apply(API::UObject* mid, uint32_t cls) {
    auto* rt = dot_rt();
    if (mid == nullptr || rt == nullptr) return;
    {
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        API::FName param = make_fname(L"SlateUI");
        memcpy(q, &param, sizeof(int32_t) * 2);
        *reinterpret_cast<void**>(q + 8) = rt;
        mid->call_function(L"SetTextureParameterValue", q);
    }
    {
        // Colour AND brightness, together, in float. TintColorAndOpacity defaults to black on
        // this asset, so this write is also what makes the dot render at all.
        float c[3];
        dot_color_for(cls, c);
        const float g = g_cfg.wrist_radar_gain;
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        API::FName param = make_fname(L"TintColorAndOpacity");
        memcpy(q, &param, sizeof(int32_t) * 2);
        auto* t = reinterpret_cast<float*>(q + 8);
        t[0] = c[0] * g; t[1] = c[1] * g; t[2] = c[2] * g; t[3] = 1.0f;
        mid->call_function(L"SetVectorParameterValue", q);
    }
    {
        alignas(16) uint8_t q[RIG_PARAM_BUF] = {0};
        API::FName param = make_fname(L"OpacityFromTexture");
        memcpy(q, &param, sizeof(int32_t) * 2);
        *reinterpret_cast<float*>(q + 8) = 1.0f;
        mid->call_function(L"SetScalarParameterValue", q);
    }
}

API::UObject* dot_spawn(API::UObject* owner, uint32_t cls, double scale, API::UObject** out_mid) {
    *out_mid = nullptr;
    // FIND *OR LOAD*, and cache for the session. find_uobject only sees objects already loaded,
    // and /Engine/BasicShapes/Sphere is present on some levels and absent on others -- on a map
    // without it dot_spawn bailed here and every contact fell back to the grenade art ("why are
    // grunts frags now"). The failure was silent because it returned before the colour path,
    // which is where the logging lives.
    static TrackedObject s_sphere;
    auto* sphere = s_sphere.get();
    if (sphere == nullptr) {
        sphere = API::get()->find_uobject<API::UObject>(L"StaticMesh /Engine/BasicShapes/Sphere.Sphere");
        if (sphere == nullptr) sphere = load_asset_by_path("/Engine/BasicShapes/Sphere.Sphere");
        if (sphere == nullptr) {
            static bool s_moaned = false;
            if (!s_moaned) {
                s_moaned = true;
                API::get()->log_info("[Halo-CampE-UEVR] RADAR: no sphere mesh (engine primitive "
                                     "absent and load failed) -- blips fall back to grenade art");
            }
            return nullptr;
        }
        s_sphere.set(sphere);
    }
    if (dot_rt() == nullptr) return nullptr;
    auto* comp = holster_marker_spawn_mesh(owner, sphere, scale);
    if (comp == nullptr) return nullptr;
    auto* base = API::get()->find_uobject<API::UObject>(
        L"MaterialInstanceConstant /Engine/EngineMaterials/Widget3DPassThrough_Opaque."
        L"Widget3DPassThrough_Opaque");
    if (base != nullptr) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        *reinterpret_cast<int32_t*>(p) = 0;
        *reinterpret_cast<void**>(p + 8) = base;
        comp->call_function(L"CreateDynamicMaterialInstance", p);
        *out_mid = *reinterpret_cast<API::UObject**>(p + 24);
    }
    if (*out_mid == nullptr) { holster_marker_show(comp, false); return nullptr; }
    dot_apply(*out_mid, cls);
    return comp;
}

// NATIVE-BLIP INVESTIGATION, ROUND 2 (doctrine at Config::tracker_mid). Snapshot-and-diff of the
// tracker's paint machinery. Targets: every descendant of the live tracker whose class is a
// MaterialInstanceDynamic or contains "MotionTrackerImage". Each target gets its object memory
// snapshotted AND every plausible embedded TArray's payload (ptr + 0 < num <= max <= 256 at
// ptr+8/+12) -- parameter values live in those heap blocks, invisible to the object diff. Two
// diff rounds ~0.5 s apart so a churn cadence shows as two hits at the same offset.
namespace tmid {
constexpr int MAX_TGT = 4;
constexpr int MAX_ARR = 8;
constexpr size_t OBJ_BYTES = 0x300;
struct Arr { size_t off = 0; uint8_t* base = nullptr; size_t bytes = 0; uint8_t snap[0x200]; };
struct Tgt {
    API::UObject* obj = nullptr;
    std::wstring  cls;
    uint8_t       snap[OBJ_BYTES];
    Arr           arr[MAX_ARR];
    int           narr = 0;
};
Tgt s_tgt[MAX_TGT];
int s_ntgt = 0;
int s_round = 0;        // rounds remaining
uint32_t s_due = 0;     // s_tick the next diff runs at

void snap_target(Tgt& t) {
    if (!IsBadReadPtr(t.obj, OBJ_BYTES)) memcpy(t.snap, t.obj, OBJ_BYTES);
    const uint8_t* base = reinterpret_cast<const uint8_t*>(t.obj);
    t.narr = 0;
    for (size_t off = 0; off + 16 <= OBJ_BYTES && t.narr < MAX_ARR; off += 8) {
        uint8_t* p = *reinterpret_cast<uint8_t* const*>(base + off);
        const int32_t num = *reinterpret_cast<const int32_t*>(base + off + 8);
        const int32_t mx  = *reinterpret_cast<const int32_t*>(base + off + 12);
        if (p == nullptr || num <= 0 || num > mx || mx > 256) continue;
        size_t bytes = (size_t)num * 16;
        if (bytes > sizeof(Arr::snap)) bytes = sizeof(Arr::snap);
        if (IsBadReadPtr(p, bytes)) continue;
        Arr& a = t.arr[t.narr++];
        a.off = off; a.base = p; a.bytes = bytes;
        memcpy(a.snap, p, bytes);
    }
}

void diff_target(Tgt& t, int round, int* lines) {
    if (IsBadReadPtr(t.obj, OBJ_BYTES)) return;
    const uint8_t* cur = reinterpret_cast<const uint8_t*>(t.obj);
    for (size_t off = 0; off + 4 <= OBJ_BYTES && *lines < 60; off += 4) {
        const uint32_t o = *reinterpret_cast<const uint32_t*>(t.snap + off);
        const uint32_t n = *reinterpret_cast<const uint32_t*>(cur + off);
        if (o == n) continue;
        ++*lines;
        API::get()->log_info("[Halo-CampE-UEVR] TRACKERMID r%d %ls +0x%03X %08X->%08X (f %.3f->%.3f)",
                             round, t.cls.c_str(), (unsigned)off, o, n,
                             *reinterpret_cast<const float*>(t.snap + off),
                             *reinterpret_cast<const float*>(cur + off));
    }
    for (int arr_idx = 0; arr_idx < t.narr && *lines < 60; ++arr_idx) {
        Arr& a = t.arr[arr_idx];
        if (IsBadReadPtr(a.base, a.bytes)) continue;
        for (size_t off = 0; off + 4 <= a.bytes && *lines < 60; off += 4) {
            const uint32_t o = *reinterpret_cast<const uint32_t*>(a.snap + off);
            const uint32_t n = *reinterpret_cast<const uint32_t*>(a.base + off);
            if (o == n) continue;
            ++*lines;
            API::get()->log_info("[Halo-CampE-UEVR] TRACKERMID r%d %ls arr@+0x%03X +0x%03X %08X->%08X (f %.3f->%.3f)",
                                 round, t.cls.c_str(), (unsigned)a.off, (unsigned)off, o, n,
                                 *reinterpret_cast<const float*>(a.snap + off),
                                 *reinterpret_cast<const float*>(a.base + off));
        }
    }
}
} // namespace tmid

void tracker_mid_probe(uint32_t tick) {
    static int s_armed_as = 0;
    if (tmid::s_round > 0) {
        if (tick < tmid::s_due) return;
        const int round = 3 - tmid::s_round;   // 1 then 2
        int lines = 0;
        for (int i = 0; i < tmid::s_ntgt; ++i) tmid::diff_target(tmid::s_tgt[i], round, &lines);
        API::get()->log_info("[Halo-CampE-UEVR] TRACKERMID round %d: %d changed dwords logged", round, lines);
        for (int i = 0; i < tmid::s_ntgt; ++i) tmid::snap_target(tmid::s_tgt[i]);
        --tmid::s_round;
        tmid::s_due = tick + 30;
        return;
    }
    if (g_cfg.tracker_mid == s_armed_as) return;
    s_armed_as = g_cfg.tracker_mid;
    if (s_armed_as == 0) return;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count();
    API::UObject* tracker = nullptr;
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o).find(L"WBP_MotionTracker") == std::wstring::npos) continue;
        if (!live_widget_instance(o)) continue;
        tracker = o;
        break;
    }
    if (tracker == nullptr) {
        API::get()->log_info("[Halo-CampE-UEVR] TRACKERMID %d: no live tracker", s_armed_as);
        return;
    }
    tmid::s_ntgt = 0;
    for (int32_t i = 0; i < nn && tmid::s_ntgt < tmid::MAX_TGT; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring cls = class_name_of(o);
        if (cls.find(L"MaterialInstanceDynamic") == std::wstring::npos
            && cls.find(L"MotionTrackerImage") == std::wstring::npos) continue;
        bool under = false;
        int depth = 0;
        for (API::UObject* p = o->get_outer(); p != nullptr && depth < 12; p = p->get_outer(), ++depth)
            if (p == tracker) { under = true; break; }
        if (!under) continue;
        tmid::Tgt& t = tmid::s_tgt[tmid::s_ntgt++];
        t.obj = o; t.cls = cls;
        tmid::snap_target(t);
        API::get()->log_info("[Halo-CampE-UEVR] TRACKERMID %d target: %ls @%p (%d arrays)",
                             s_armed_as, cls.c_str(), (void*)o, t.narr);
    }
    if (tmid::s_ntgt == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] TRACKERMID %d: no paint targets under tracker", s_armed_as);
        return;
    }
    tmid::s_round = 2;
    tmid::s_due = tick + 30;
}

// The radar's per-contact visuals (doctrine at their use in wristhud_place). File scope rather than
// a function-local static so the HUD's off edge can hide every dot it spawned.
struct BlipViz {
    TrackedObject mesh;
    TrackedObject mid;                   // colour MID when is_dot (team/gain re-applies)
    bool     is_dot = false;
    uint32_t id = 0;
    uint32_t team = 0;    // the SPECIES ID (object's leading dword)
    float    sfwd = 0.0f, srgt = 0.0f;   // smoothed panel coords, room metres
    uint32_t seen_tick = 0;              // id present in the latest publish
    uint32_t moving_tick = 0;            // last publish that said "moving"
};
BlipViz s_bv[MAX_BLIPS];

} // namespace

// ================================================================================================
// HUD PLACEMENT ON THE WEAPON (hudplacement=1). The panels ride the drawn weapon instead of the
// forearms. The game tick resolves whether a first-person weapon is in hand and which actor it is
// (class-name checks are tick work, not render work); the render thread reads the anchor transform
// each frame, right where the wrist placement runs, and places every panel from it.
//
// THE ANCHOR, per hudwpnanchor (Config.hpp):
//   1  RootComponent of the FP weapon actor. Rig.cpp: the weapon actor is attached to the arms rig at
//      socket PrimaryWeapon, so its root follows that socket under either arm driver.
//   2  GetSocketLocation / GetSocketRotation("PrimaryWeapon") on the arms rig component: the posed
//      skeleton, read at the moment of the call. Under armdriver 1 the render-rate rig re-apply has
//      already written this frame's rig transform earlier in the same callback; under armdriver 3 the
//      render-output instruments (STOMPLOG point 12, SOCKROT) record this socket as the drawn gun.
//   3  the palette weapon pose rotation (armdriver 3 only) at the socket position of 2.
// ================================================================================================
namespace {

std::atomic<void*> s_wpn_root{nullptr};   // the FP weapon actor's RootComponent, published per tick
std::atomic<bool>  s_wpn_live{false};     // a first-person weapon is in hand (Rig.cpp route check)

struct WhWpnAnchor { Vec3 pos{}; Quat q{0.0f, 0.0f, 0.0f, 1.0f}; int how = 0; };

// Game tick. Nothing while hudplacement is 0.
void wh_weapon_anchor_tick() {
    if (g_cfg.hud_placement != 1) {
        s_wpn_live.store(false, std::memory_order_relaxed);
        s_wpn_root.store(nullptr, std::memory_order_relaxed);
        return;
    }
    const bool live = fp_weapon_route_alive();
    s_wpn_root.store(live ? static_cast<void*>(fp_weapon_root()) : nullptr, std::memory_order_relaxed);
    s_wpn_live.store(live, std::memory_order_relaxed);
}

// GetSocketRotation(FName) -> FRotator: FName at offset 0, the three doubles at offset 8.
bool wh_socket_rot(API::UObject* comp, const wchar_t* socket, Vec3* out_pyr) {
    if (comp == nullptr || out_pyr == nullptr) return false;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    API::FName name = make_fname(socket);
    memcpy(params, &name, sizeof(int32_t) * 2);
    comp->call_function(L"GetSocketRotation", params);
    auto* d = reinterpret_cast<double*>(params + 8);
    if (!std::isfinite(d[0]) || !std::isfinite(d[1]) || !std::isfinite(d[2])) return false;
    *out_pyr = Vec3{(float)d[0], (float)d[1], (float)d[2]};
    return true;
}

// Render thread. The weapon's world transform by the configured anchor; false = no usable anchor.
bool wh_weapon_anchor(WhWpnAnchor* out) {
    if (!s_wpn_live.load(std::memory_order_relaxed)) return false;
    int how = g_cfg.hud_wpn_anchor;
    if (how == 3 && !palette_weapon_mode()) how = 2;
    if (how == 1) {
        auto* root = static_cast<API::UObject*>(s_wpn_root.load(std::memory_order_relaxed));
        Vec3 p{}, r{};
        if (root == nullptr || !call_ret_vec3(root, L"K2_GetComponentLocation", &p) ||
            !call_ret_vec3(root, L"K2_GetComponentRotation", &r)) return false;
        out->pos = p; out->q = rotator_to_quat(r.x, r.y, r.z); out->how = 1;
        return true;
    }
    auto* rig = static_cast<API::UObject*>(g_rig_component.load());
    Vec3 p{};
    if (rig == nullptr || !call_socket_location(rig, L"PrimaryWeapon", &p)) return false;
    if (how == 3) {
        const Quat q{g_dbg_pose_w_x.load(std::memory_order_relaxed), g_dbg_pose_w_y.load(std::memory_order_relaxed),
                     g_dbg_pose_w_z.load(std::memory_order_relaxed), g_dbg_pose_w_w.load(std::memory_order_relaxed)};
        const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (!(n > 0.5f) || !std::isfinite(n)) return false;
        out->pos = p; out->q = Quat{q.x / n, q.y / n, q.z / n, q.w / n}; out->how = 3;
        return true;
    }
    Vec3 r{};
    if (!wh_socket_rot(rig, L"PrimaryWeapon", &r)) return false;
    out->pos = p; out->q = rotator_to_quat(r.x, r.y, r.z); out->how = 2;
    return true;
}

// Render thread. One panel onto its weapon slot. The tracker slot also hands back the ROOM-space
// frame the radar dots are laid out in, the same frame the wrist path hands them.
void wh_place_on_weapon(API::UObject* comp, const std::wstring& match, const WhWpnAnchor& a,
                        const Vec3& hpos, int* other_idx, bool* tr_have, Vec3* tr_at, Vec3* tr_F,
                        Vec3* tr_U, float* tr_p, float* tr_y, float* tr_r) {
    float slot[6];
    const bool tracker = match.find(L"MotionTracker") != std::wstring::npos;
    const float* src = tracker ? g_cfg.hud_wpn_tracker
                     : (match.find(L"ShieldHealth") != std::wstring::npos) ? g_cfg.hud_wpn_shield
                     : (match.find(L"WeaponCradle") != std::wstring::npos) ? g_cfg.hud_wpn_ammo
                     : g_cfg.hud_wpn_grenade;
    for (int i = 0; i < 6; ++i) slot[i] = src[i];
    const bool other = !tracker && match.find(L"ShieldHealth") == std::wstring::npos &&
                       match.find(L"WeaponCradle") == std::wstring::npos &&
                       match.find(L"GrenadeCradle") == std::wstring::npos;
    if (other) slot[2] -= g_cfg.hud_wpn_gap * (float)(++(*other_idx));

    const Quat qs = quat_mul(a.q, rotator_to_quat(slot[3], slot[4], slot[5]));
    const Vec3 lo = quat_rotate(a.q, Vec3{slot[0], slot[1], slot[2]});
    const Vec3 w{a.pos.x + lo.x, a.pos.y + lo.y, a.pos.z + lo.z};
    float p = 0.0f, y = 0.0f, r = 0.0f;
    quat_to_rotator(qs.x, qs.y, qs.z, qs.w, &p, &y, &r);
    holster_marker_place_rot(comp, w, p, y, r);
    holster_marker_scale(comp, (double)g_cfg.hud_wpn_scale);

    if (tracker && !*tr_have) {
        const Vec3 r0 = holster_world_to_room(w, hpos);
        auto room_dir = [&](const Vec3& dw) {
            const Vec3 r1 = holster_world_to_room(Vec3{w.x + dw.x * 10.0f, w.y + dw.y * 10.0f, w.z + dw.z * 10.0f}, hpos);
            Vec3 o{r1.x - r0.x, r1.y - r0.y, r1.z - r0.z};
            const float l = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z);
            if (l > 1e-6f) { o.x /= l; o.y /= l; o.z /= l; }
            return o;
        };
        *tr_have = true;
        *tr_at = r0;
        *tr_F = room_dir(quat_rotate(qs, Vec3{1.0f, 0.0f, 0.0f}));
        *tr_U = room_dir(quat_rotate(qs, Vec3{0.0f, 0.0f, 1.0f}));
        *tr_p = p; *tr_y = y; *tr_r = r;
    }

    if (g_cfg.hud_wpn_log) {
        static uint32_t s_n = 0;
        if ((s_n++ % 90u) == 0u) {
            const Vec3 ax = quat_rotate(a.q, Vec3{1.0f, 0.0f, 0.0f});
            const Vec3 ay = quat_rotate(a.q, Vec3{0.0f, 1.0f, 0.0f});
            const Vec3 az = quat_rotate(a.q, Vec3{0.0f, 0.0f, 1.0f});
            API::get()->log_info("[Halo-CampE-UEVR] HUDWPN anchor=%d pos=(%.1f %.1f %.1f) X=(%.2f %.2f %.2f) "
                                 "Y=(%.2f %.2f %.2f) Z=(%.2f %.2f %.2f) | %ls at (%.1f %.1f %.1f) rot p%.1f y%.1f r%.1f",
                                 a.how, a.pos.x, a.pos.y, a.pos.z, ax.x, ax.y, ax.z, ay.x, ay.y, ay.z, az.x, az.y, az.z,
                                 match.c_str(), w.x, w.y, w.z, p, y, r);
        }
    }
}

} // namespace

// DISCOVERY ONLY, once per game tick: census, hosting, and the per-tick colour chain. Placement
// deliberately does NOT live here -- see wristhud_place().
void wristhud_tick() {
    ++s_tick;
    tracker_dump_probe();
    tracker_mid_probe(s_tick);
    if (!g_cfg.wrist_hud) return;
    wh_weapon_anchor_tick();   // hudplacement=1: the weapon anchor for the render thread
    parse_slots();

    auto* owner = API::get()->get_local_pawn(0);
    if (owner == nullptr) return;
    sweep(owner);
}

// PLACEMENT, at RENDER rate, from the stereo callback. The panels used to be positioned once per
// game tick against the camera as it stood at that moment; the headset draws several frames per
// tick, so every frame between ticks rendered the panel against a camera that had already moved
// and the whole HUD swam against the world whenever the player did ("the huds jutter like crazy
// when im moving"). Standing still it looked fine, which is why it survived this long. The weapon
// rig already solved exactly this by re-applying on the render path; this is the same treatment.
void wristhud_place() {
    static bool s_was_on = false;
    if (!g_cfg.wrist_hud) {
        // OFF EDGE: nothing below runs any more, so anything left showing would freeze in the
        // world at its last pose. Hide the panels and every radar dot, and give the ammo box its
        // visibility back. The hosted game widgets themselves stay off the flat HUD until the next
        // level load (the re-parse note in parse_slots says why that is a bigger job).
        if (s_was_on) {
            for (int i = 0; i < s_slot_count; ++i)
                if (auto* c = s_slots[i].comp.get()) holster_marker_show(c, false);
            for (auto& v : s_bv) {
                if (auto* m = v.mesh.get()) holster_marker_show(m, false);
                v.id = 0;
            }
            wh_cradle_restore();
        }
        s_was_on = false;
        return;
    }
    s_was_on = true;
    auto* owner = API::get()->get_local_pawn(0);
    if (owner == nullptr) return;
    // The off hand's forearm. Hidden in stick mode (vehicles, cutscenes, death),
    // where the hands are not the player's own.
    const bool hide = g_stick_mode_active.load(std::memory_order_relaxed)
                   || (g_cfg.wrist_hud_trigger && !g_wristhud_lt.load(std::memory_order_relaxed));
    const auto gidx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                          : API::VR::get_left_controller_index();
    const auto aidx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();
    Vec3 gpos{}; Quat grot{};
    Vec3 apos{}; Quat arot{};
    Vec3 hpos{}; Quat hrot{};
    const bool have_head = !hide && get_pose(API::VR::get_hmd_index(), &hpos, &hrot, /*use_aim=*/false);
    const bool have_l = have_head
        && get_pose(gidx, &gpos, &grot, /*use_aim=*/false)
        && !(std::fabs(gpos.x) < 1e-6f && std::fabs(gpos.y) < 1e-6f && std::fabs(gpos.z) < 1e-6f);
    const bool have_r = have_head
        && get_pose(aidx, &apos, &arot, /*use_aim=*/false)
        && !(std::fabs(apos.x) < 1e-6f && std::fabs(apos.y) < 1e-6f && std::fabs(apos.z) < 1e-6f);

    // ON THE WEAPON (hudplacement=1). With it 0, weapon_place and wpn_hide are false and nothing below
    // changes: every panel takes the wrist path exactly as before.
    const bool wpn_mode = g_cfg.hud_placement == 1;
    const bool in_menu = wpn_mode && g_menu_active.load(std::memory_order_relaxed);
    WhWpnAnchor wpn{};
    const bool weapon_place = wpn_mode && have_head && !in_menu && wh_weapon_anchor(&wpn);
    const bool wpn_hide = wpn_mode && !weapon_place && (in_menu || g_cfg.hud_wpn_fallback == 0);
    int wpn_other = 0;

    // Local orientation trims composed as QUATERNIONS on the controller pose -- correct rotation
    // composition, not rotator addition. Each wrist has its own trim set: the two forearms are
    // mirror poses and shared numbers fit neither.
    const Quat qoff_l = rotator_to_quat(g_cfg.wrist_hud_rot[0], g_cfg.wrist_hud_rot[1], g_cfg.wrist_hud_rot[2]);
    const Quat qoff_r = rotator_to_quat(g_cfg.wrist_hud_rot_r[0], g_cfg.wrist_hud_rot_r[1], g_cfg.wrist_hud_rot_r[2]);

    // The tracker panel's frame, stashed while the loop places it, consumed by the blips below.
    bool tr_have = false;
    Vec3 tr_at{}, tr_F{}, tr_U{};                  // ROOM-space basis (see the blip block)
    float tr_p = 0.0f, tr_y = 0.0f, tr_r = 0.0f;   // the panel's placed rotation, for RADAR-PL

    int idx_l = 0, idx_r = 0;
    static uint32_t s_bind_n = 0;
    const bool bind_now = (s_bind_n++ % 6) == 0;   // the reticule's cadence for the same call (three reflected calls and a full-name walk)
    for (int si = 0; si < s_slot_count; ++si) {
        auto* comp = s_slots[si].comp.get();
        if (comp == nullptr) continue;
        // The colour chain, both halves (doctrine at their exports): SlateUI re-bound every 6th
        // frame (also the crash guard -- an unbound sample faults in the translucency pass) and the
        // pre-exposure tint gain that keeps the panels from tonemapping to black.
        if (bind_now) bind_widget_slate_ui(comp);
        wristhud_apply_widget_tint(comp, s_slots[si].right ? g_cfg.wrist_hud_gain_r : 1.0f, false);
        const bool right = s_slots[si].right;
        const bool have = weapon_place ? true : (wpn_hide ? false : (right ? have_r : have_l));
        const int  sidx = right ? idx_r++ : idx_l++;
        if (!have) { holster_marker_show(comp, false); continue; }
        holster_marker_show(comp, true);
        if (s_slots[si].match.find(L"WeaponCradle") != std::wstring::npos) {
            if (g_wristhud_hide_cradle.load(std::memory_order_relaxed)) wh_cradle_write_zero(s_slots[si].widget.get());
            else wh_cradle_restore();
        }
        if (weapon_place) {
            wh_place_on_weapon(comp, s_slots[si].match, wpn, hpos, &wpn_other,
                               &tr_have, &tr_at, &tr_F, &tr_U, &tr_p, &tr_y, &tr_r);
            continue;
        }

        const Vec3& cpos = right ? apos : gpos;
        const Quat& crot = right ? arot : grot;
        const float* off = right ? g_cfg.wrist_hud_off_r : g_cfg.wrist_hud_off;
        const float  gap = right ? g_cfg.wrist_hud_gap_r : g_cfg.wrist_hud_gap;

        // Controller-local offset, slot-stacked along the forearm axis per side.
        const Vec3 loc{off[0], off[1], off[2] + (float)sidx * gap};
        const Vec3 lw = quat_rotate(crot, loc);
        const Vec3 at{cpos.x + lw.x, cpos.y + lw.y, cpos.z + lw.z};

        // Orientation: same direction-vector extraction as the rotating mag -- forward and up
        // pushed through room_to_world as differences, yaw/pitch from forward, roll recovered
        // from where up landed. One frame-math implementation, everywhere.
        const Quat qe = quat_mul(crot, right ? qoff_r : qoff_l);
        auto wdir = [&](const Vec3& d) {
            const Vec3 w0 = holster_room_to_world(at, hpos);
            const Vec3 w1 = holster_room_to_world(Vec3{at.x + d.x, at.y + d.y, at.z + d.z}, hpos);
            Vec3 out{w1.x - w0.x, w1.y - w0.y, w1.z - w0.z};
            const float l = std::sqrt(out.x * out.x + out.y * out.y + out.z * out.z);
            if (l > 1e-4f) { out.x /= l; out.y /= l; out.z /= l; }
            return out;
        };
        const Vec3 F = wdir(quat_forward(qe));
        const Vec3 U = wdir(quat_rotate(qe, Vec3{0.0f, 1.0f, 0.0f}));
        const float yawr   = std::atan2(F.y, F.x);
        const float pitchr = std::asin(std::fmax(-1.0f, std::fmin(1.0f, F.z)));
        const float cy = std::cos(yawr), sy = std::sin(yawr);
        const float cp = std::cos(pitchr), sp = std::sin(pitchr);
        const Vec3 right0{-sy, cy, 0.0f};
        const Vec3 up0{-sp * cy, -sp * sy, cp};
        const float rollr = std::atan2(U.x * right0.x + U.y * right0.y + U.z * right0.z,
                                       U.x * up0.x + U.y * up0.y + U.z * up0.z);
        holster_marker_place_rot(comp, holster_room_to_world(at, hpos),
                                 pitchr * RAD2DEG, yawr * RAD2DEG, rollr * RAD2DEG);
        holster_marker_scale(comp, (double)g_cfg.wrist_hud_scale);
        if (right && sidx == 0) {
            // ROOM-space axes, deliberately NOT the world-space F/U above. The blips offset a
            // ROOM-space position and then push the sum through room_to_world, so the offset
            // must be room-space too. Mixing them made room_to_world rotate the offset a SECOND
            // time (by the camera yaw, plus the swizzle) -- the dot then wandered around the
            // disc as the player turned and changed radius, while its panel-space coordinates
            // logged as dead constant. That frame mismatch, not any sign or reference frame, is
            // what every earlier radar "fix" was chasing.
            tr_have = true;
            tr_at = at;
            tr_F = quat_forward(qe);
            tr_U = quat_rotate(qe, Vec3{0.0f, 1.0f, 0.0f});
            tr_p = pitchr * RAD2DEG; tr_y = yawr * RAD2DEG; tr_r = rollr * RAD2DEG;
        }
    }

    // ---- THE WRIST RADAR'S BLIPS (doctrine at the exports in BlamDrive.hpp). The hosted
    // tracker's native feed is severed by design, so the dots are ours: the sim publishes
    // relative Blam offsets + team; here they rotate into the FACING frame (blam -> UE -> room
    // via the same yaw the whole room transform is pinned to, then decomposed against the head's
    // horizontal forward) and land on the panel plane. Frag mesh = human, plasma = covenant --
    // each faction painted with its own ordnance.
    {
        // Per-CONTACT visuals, keyed on the published identity: the publish order compacts as
        // contacts drop in and out of range, so index i is a different enemy from one tick to
        // the next and any per-index state smears. Each contact gets its own mesh, an EMA on its
        // panel position (the source data steps at the sim publish cadence; the panel is
        // pose-smooth, so raw steps read as jitter), and STICKY visibility -- the moving flag
        // blinks at its threshold, and a dot that holds for ~a second after the last movement
        // reads like the real tracker instead of a strobe. The same hold expires a settling
        // corpse's ragdoll blip quickly.
        // (BlipViz and s_bv live at file scope, so switching the HUD off can hide the dots too.)
        constexpr uint32_t HOLD_TICKS = 160;     // ~2 s visible after the last movement -- a
                                                 // PATROLLING NPCS walk stop-and-go, and the video
                                                 // caught its dot blinking through every pause
        constexpr uint32_t STALE_TICKS = 40;     // id gone from the publish this long = free the slot
        const bool rtest = g_cfg.wrist_radar_test != 0;
        const int nblips = (g_cfg.wrist_radar && tr_have)
                         ? (rtest ? 1 : g_blip_count.load(std::memory_order_relaxed)) : 0;
        auto* mf = host::g_holster_state.mesh_frag->get();
        auto* mp = host::g_holster_state.mesh_plasma->get();
        const float half = g_cfg.wrist_hud_draw * (weapon_place ? g_cfg.hud_wpn_scale : g_cfg.wrist_hud_scale) / 200.0f;   // panel half-width, room metres
        // Panel-plane axes.
        const Vec3 R{tr_F.y * tr_U.z - tr_F.z * tr_U.y,
                     tr_F.z * tr_U.x - tr_F.x * tr_U.z,
                     tr_F.x * tr_U.y - tr_F.y * tr_U.x};
        // THE FIELD IS AIM-UP, like classic Halo. SETTLED BY ASKING THE MAN IN THE HEADSET
        // (2026-08-29): the art's big centre fan "always points up", so the disc does not
        // rotate and the fan is not a compass needle. An earlier read of two screenshots
        // concluded the opposite (world-fixed disc, rotating wedge) -- that inference did not
        // control for the panel's own roll and oblique viewing angle, and it was wrong.
        //
        // Aim-relative was in fact right all along, but it was never once tested against a
        // WORKING dot plane: the tilt bug (see wrist_radar_tilt) collapsed the whole
        // forward/back axis into invisible depth through every aim-relative attempt, which is
        // what "all over the place depending on how I'm turned" actually was.
        const float aim_rad = g_desired_yaw.load(std::memory_order_relaxed) * DEG2RAD;

        for (int i = 0; i < nblips; ++i) {
            const uint32_t id = rtest ? 0xC0FFEEu : g_blip_id[i].load(std::memory_order_relaxed);
            if (id == 0) continue;
            // STAR MODE 1: fixed world bearing. MODE 2: the star is planted at the bearing the
            // player is CURRENTLY AIMING, so a correctly calibrated field draws it inside the
            // native facing wedge at every facing -- a NULL TEST. It needs only "on it" or
            // "off it" from the headset instead of an angle estimate, and angle estimates
            // against an oblique wrist panel are what made a night of readings unfittable.
            // A field turning the wrong way separates at DOUBLE rate as the player turns, which
            // is unmissable; a constant gap is just the trim.
            // Placed at the AIM bearing in the same convention the renderer subtracts, so a
            // healthy chain parks it dead ahead (inside the fan) at every facing. NOTE this
            // cannot test the convention itself -- both sides use the same value -- it tests the
            // PLANE and the CENTRING. Aiming at a real enemy is what tests the convention.
            const float star_a = aim_rad;
            const float bdx = rtest ? (g_cfg.wrist_radar_test >= 2 ? 5.0f * std::cos(star_a) : 5.0f)
                                    : g_blip_dx[i].load(std::memory_order_relaxed);
            const float bdy = rtest ? (g_cfg.wrist_radar_test >= 2 ? 5.0f * std::sin(star_a) : 0.0f)
                                    : g_blip_dy[i].load(std::memory_order_relaxed);
            const float dist = std::sqrt(bdx * bdx + bdy * bdy);
            if (dist < 1e-4f) continue;
            // Bearing MINUS aim: both are in the same angle convention, fitted against logged
            // numbers at a moment he was provably aiming at a target (2026-08-28 18:53, blam
            // (-0.17,-2.73) = bearing -93.6 deg, aim -95.3 deg -> difference 1.7 deg, i.e. dead
            // ahead). Positive = target LEFT of aim; rgt_c = -sin so left renders left.
            const float rel_ang = std::atan2(bdy, bdx) - g_cfg.wrist_radar_aimsign * aim_rad;
            const float fwd_c = std::cos(rel_ang);             // along the aim = panel up
            float rgt_c = -std::sin(rel_ang);                  // right of the aim = panel right
            if (g_cfg.wrist_radar_flip) rgt_c = -rgt_c;
            const float rr = (dist / 8.2f) * half;             // 25 m -> panel edge
            // The steer-by-feel in-plane corrector (doctrine at Config::wrist_radar_rot).
            const float th = g_cfg.wrist_radar_rot * DEG2RAD;
            const float ct = std::cos(th), st = std::sin(th);
            const float tfwd = (ct * fwd_c - st * rgt_c) * rr;
            const float trgt = (st * fwd_c + ct * rgt_c) * rr;

            // The chain, in numbers (doctrine at Config::wrist_radar_log). EVERY contact, ~1 Hz:
            // one-slot logging made the fits unfalsifiable -- slot 0 is whichever contact
            // published first, not the one the player is describing.
            if (g_cfg.wrist_radar_log) {
                static uint32_t s_rlog_tick = 0;
                static bool     s_rlog_round = false;
                if (i == 0) {
                    s_rlog_round = (s_tick - s_rlog_tick >= 90);
                    if (s_rlog_round) s_rlog_tick = s_tick;
                }
                if (s_rlog_round) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] RADAR[%d/%d] id=%08X aim=%.1fdeg blam=(%.2f,%.2f) "
                        "rel=%.1fdeg fwd=%.2f rgt=%.2f dist=%.1f mv=%d raw=0x%02X sp=%.2f dtm=%d run=%d",
                        i, nblips, id,
                        g_desired_yaw.load(std::memory_order_relaxed), bdx, bdy,
                        rel_ang * RAD2DEG, fwd_c, rgt_c, dist,
                        (int)(rtest || g_blip_moving[i].load(std::memory_order_relaxed)),
                        g_blip_raw[i].load(std::memory_order_relaxed),
                        g_blip_speed[i].load(std::memory_order_relaxed),
                        g_blip_dtm[i].load(std::memory_order_relaxed),
                        g_blip_run[i].load(std::memory_order_relaxed));
                    // THE PLACEMENT CHAIN. Panel-space coords proved constant while the dot
                    // visibly moved around the disc, so the fault is between panel space and
                    // the world: the quad's placed rotation (roll extraction degenerates as the
                    // normal approaches vertical -- looking down at a wrist IS that pose) and
                    // the axes the offset is built from.
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] RADAR-PL panel p=%.1f y=%.1f r=%.1f | F=(%.2f,%.2f,%.2f) "
                        "U=(%.2f,%.2f,%.2f) R=(%.2f,%.2f,%.2f) half=%.3f",
                        tr_p, tr_y, tr_r,
                        tr_F.x, tr_F.y, tr_F.z, tr_U.x, tr_U.y, tr_U.z, R.x, R.y, R.z, half);
                    // The placement axes against the VIEW direction. A healthy panel: F (the
                    // quad normal) dots near +/-1 with the eye-to-panel line, R and U near 0
                    // (in-plane). The published circle is proven perfect, so if the visual
                    // collapses one axis, one of these dots is the confession.
                    Vec3 vv{tr_at.x - hpos.x, tr_at.y - hpos.y, tr_at.z - hpos.z};
                    const float vl = std::sqrt(vv.x * vv.x + vv.y * vv.y + vv.z * vv.z);
                    if (vl > 1e-4f) { vv.x /= vl; vv.y /= vl; vv.z /= vl; }
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] RADAR-AX F.v=%.2f U.v=%.2f R.v=%.2f |R|=%.2f",
                        tr_F.x * vv.x + tr_F.y * vv.y + tr_F.z * vv.z,
                        tr_U.x * vv.x + tr_U.y * vv.y + tr_U.z * vv.z,
                        R.x * vv.x + R.y * vv.y + R.z * vv.z,
                        std::sqrt(R.x * R.x + R.y * R.y + R.z * R.z));
                }
            }

            BlipViz* v = nullptr;
            for (int k = 0; k < MAX_BLIPS; ++k) if (s_bv[k].id == id) { v = &s_bv[k]; break; }
            if (v == nullptr) {
                for (int k = 0; k < MAX_BLIPS; ++k)
                    if (s_bv[k].id == 0 || s_tick - s_bv[k].seen_tick > STALE_TICKS) { v = &s_bv[k]; break; }
                if (v == nullptr) continue;
                v->id = id;
                v->sfwd = tfwd; v->srgt = trgt;   // snap on first sight -- nothing to smooth from
                v->moving_tick = 0;
            } else {
                // Glide only across SMALL steps (the publish cadence). A snap turn jumps the
                // target a long way around the ring, and an EMA takes the chord -- the dot
                // visibly cuts through the middle of the tracker. Discrete jump, discrete move.
                const float jf = tfwd - v->sfwd, jr = trgt - v->srgt;
                if (jf * jf + jr * jr > (0.4f * rr) * (0.4f * rr)) {
                    v->sfwd = tfwd; v->srgt = trgt;
                } else {
                    v->sfwd += 0.25f * jf;
                    v->srgt += 0.25f * jr;
                }
            }
            // Name this species the first time it is seen: one actor sweep per species, ever.
            if (!rtest) {
                const uint32_t ty = g_blip_type[i].load(std::memory_order_relaxed);
                if (ty != 0 && species_name(ty) == nullptr) {
                    bool known = false;
                    for (int q2 = 0; q2 < s_species_n; ++q2) if (s_species[q2].cls == ty) known = true;
                    if (!known)
                        species_resolve(ty, blam_to_ue(g_blip_wx[i].load(std::memory_order_relaxed),
                                                       g_blip_wy[i].load(std::memory_order_relaxed),
                                                       g_blip_wz[i].load(std::memory_order_relaxed)));
                }
            }
            v->seen_tick = s_tick;
            if (rtest || g_blip_moving[i].load(std::memory_order_relaxed)) v->moving_tick = s_tick;
            v->team = rtest ? 0u : g_blip_type[i].load(std::memory_order_relaxed);
        }

        for (int k = 0; k < MAX_BLIPS; ++k) {
            BlipViz& v = s_bv[k];
            auto* m = v.mesh.get();
            const bool live = v.id != 0 && s_tick - v.seen_tick <= STALE_TICKS;
            if (!live) v.id = 0;
            const bool show = live && v.moving_tick != 0 && s_tick - v.moving_tick <= HOLD_TICKS;
            if (!show) {
                if (m != nullptr) holster_marker_show(m, false);
                continue;
            }
            // A dot's sphere is a 100 cm primitive where the grenade meshes are hand-sized, so
            // the two visual types scale differently off the same wristradarblip knob.
            static uint32_t s_last_team[MAX_BLIPS] = {0,0,0,0,0,0,0,0,0,0,0,0};
            static float s_last_gain = -1.0f;
            // On the weapon the dots shrink with the panel; on the wrists the factor is exactly 1.
            const double wpn_blip = weapon_place ? (double)g_cfg.hud_wpn_scale / (double)std::fmax(g_cfg.wrist_hud_scale, 0.001f) : 1.0;
            const double dscale = (double)g_cfg.wrist_radar_blip * 2.5 * wpn_blip;
            const double mscale = (double)g_cfg.wrist_radar_blip * 12.5 * wpn_blip;
            if (m == nullptr) {
                if (owner == nullptr) continue;
                API::UObject* mid = nullptr;
                m = dot_spawn(owner, v.team, dscale, &mid);
                v.is_dot = (m != nullptr);
                if (m == nullptr) {
                    // Colour chain unavailable -- the grenade meshes are the working fallback.
                    auto* mesh = mf;   // fallback art only, species-blind
                    if (mesh == nullptr) continue;
                    m = holster_marker_spawn_mesh(owner, mesh, mscale);
                }
                if (m == nullptr) continue;
                v.mesh.set(m);
                if (mid != nullptr) v.mid.set(mid);
                s_last_team[k] = v.team;
            } else if (v.team != s_last_team[k]) {
                if (v.is_dot) dot_apply(v.mid.get(), v.team);
                else {
                    auto* mesh = mf;
                    if (mesh != nullptr) holster_marker_set_mesh(m, mesh);
                }
                s_last_team[k] = v.team;
            }
            static int s_last_gen = -1;
            if (v.is_dot && (s_last_gain != g_cfg.wrist_radar_gain || s_last_gen != g_cfg.blip_color_gen))
                dot_apply(v.mid.get(), v.team);
            if (k == MAX_BLIPS - 1) {
                s_last_gain = g_cfg.wrist_radar_gain;
                s_last_gen = g_cfg.blip_color_gen;
            }
            // The DOT PLANE: R stays (its rendering is field-proven -- the horizontal half of the
            // swing test tracked perfectly), the vertical axis is U pitched about R by the tilt
            // knob so it lies in the ART's plane instead of running into the viewer's eye
            // (doctrine at Config::wrist_radar_tilt). The lift rides the tilted normal.
            const float tt = g_cfg.wrist_radar_tilt * DEG2RAD;
            const float ctt = std::cos(tt), stt = std::sin(tt);
            const Vec3 U2{ctt * tr_U.x + stt * tr_F.x,
                          ctt * tr_U.y + stt * tr_F.y,
                          ctt * tr_U.z + stt * tr_F.z};
            const Vec3 F2{ctt * tr_F.x - stt * tr_U.x,
                          ctt * tr_F.y - stt * tr_U.y,
                          ctt * tr_F.z - stt * tr_U.z};
            // The orbit centre rides the centre knob (doctrine at Config::wrist_radar_center).
            const float ccr = g_cfg.wrist_radar_center[0] * half;
            const float ccu = g_cfg.wrist_radar_center[1] * half;
            const Vec3 bp{tr_at.x + R.x * (ccr + v.srgt) + U2.x * (ccu + v.sfwd) + F2.x * 0.006f,
                          tr_at.y + R.y * (ccr + v.srgt) + U2.y * (ccu + v.sfwd) + F2.y * 0.006f,
                          tr_at.z + R.z * (ccr + v.srgt) + U2.z * (ccu + v.sfwd) + F2.z * 0.006f};
            holster_marker_show(m, true);
            holster_marker_place(m, holster_room_to_world(bp, hpos));
            holster_marker_scale(m, v.is_dot ? dscale : mscale);
        }
    }
}

// ---- THE RADAR BLIPS, published by the sim-thread scan below.
std::atomic<int>   g_blip_count{0};
std::atomic<float> g_blip_dx[MAX_BLIPS], g_blip_dy[MAX_BLIPS];
std::atomic<int>   g_blip_team[MAX_BLIPS];
std::atomic<bool>  g_blip_moving[MAX_BLIPS];
std::atomic<uint32_t> g_blip_id[MAX_BLIPS];
std::atomic<uint32_t> g_blip_raw[MAX_BLIPS];
std::atomic<uint32_t> g_blip_type[MAX_BLIPS];
std::atomic<float> g_blip_wx[MAX_BLIPS], g_blip_wy[MAX_BLIPS], g_blip_wz[MAX_BLIPS];
std::atomic<float> g_blip_speed[MAX_BLIPS];
std::atomic<int>   g_blip_dtm[MAX_BLIPS];
std::atomic<int>   g_blip_run[MAX_BLIPS];

namespace {

using clock::now_ms;

// ---- BLIPDUMP (wrist-radar survey, doctrine in Config.hpp). SIM THREAD, one shot per value
// change. Walks the object table the same way resolve_object_by_datum does, collects everything
// with a plausible position within ~36 m of the player, and dumps each header -- the diff between
// a marines-only capture and a covenant-only capture names the team byte.
void blip_dump_probe(uintptr_t player_obj) {
    // BlamDrive.cpp's own helpers, through the bridge: the same objects under the same names.
    const auto read_ptr = host::g_blamdrive_state.read_ptr;
    const uint32_t& g_tls_index = *host::g_blamdrive_state.tls_index;

    static int s_armed_as = 0;
    if (g_cfg.blip_dump == s_armed_as) return;
    if (IsBadReadPtr((const void*)(player_obj + 0x20), 12)) return;
    s_armed_as = g_cfg.blip_dump;
    const float* pp = (const float*)(player_obj + 0x20);

    const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
    uintptr_t block = 0, ctx = 0, table = 0;
    if (tls_array == 0 ||
        !read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &block) || block == 0 ||
        !read_ptr(block + 0x20, &ctx) || ctx == 0 ||
        !read_ptr(ctx + 0x50, &table) || table == 0) {
        API::get()->log_info("[Halo-CampE-UEVR] BLIPDUMP: table chain unresolved");
        return;
    }

    // 0x400 per object: the 0x100 sweep produced only spawn-order artifacts (the E1-vs-E2 salt
    // at +0xCC ages the object, it does not side it), so the team field lives deeper. The player
    // dumps too, labeled SELF -- its team must equal the marines', which prunes candidates hard.
    auto dump_obj = [&](uintptr_t o, int idx, float dist_m) {
        const uintptr_t span = !IsBadReadPtr((const void*)o, 0x400) ? 0x400 : 0x100;
        const uint32_t* d = (const uint32_t*)o;
        char buf[352];
        for (uintptr_t off = 0; off < span; off += 0x80) {
            int n = 0;
            for (int i = (int)(off / 4); i < (int)(off / 4) + 32 && n < (int)sizeof(buf) - 12; ++i)
                n += snprintf(buf + n, sizeof(buf) - n, "%08X ", d[i]);
            API::get()->log_info("[Halo-CampE-UEVR] BLIPDUMP cap=%d idx=%d dist=%.1fm +0x%03X | %s",
                                 g_cfg.blip_dump, idx, dist_m, (unsigned)off, buf);
        }
    };
    if (!IsBadReadPtr((const void*)player_obj, 0x100)) dump_obj(player_obj, -1, 0.0f);   // SELF
    // 24 hits, not 12: the table's low indices are load-time scenery, and a 12-object budget
    // filled with it before ever reaching the late-spawned actors the survey exists to catch.
    int hits = 0;
    for (int idx = 0; idx < 4096 && hits < 24; ++idx) {
        uintptr_t o = 0;
        if (!read_ptr(table + (uintptr_t)idx * 24 + 0x10, &o) || o == 0) continue;
        if (o == player_obj || IsBadReadPtr((const void*)(o + 0x20), 12)) continue;
        const float* q = (const float*)(o + 0x20);
        const float dx = q[0] - pp[0], dy = q[1] - pp[1], dz = q[2] - pp[2];
        const float d2 = dx * dx + dy * dy + dz * dz;
        if (!(d2 == d2) || d2 > 12.0f * 12.0f) continue;   // NaN or beyond ~36 m
        if (IsBadReadPtr((const void*)o, 0x100)) continue;
        ++hits;
        dump_obj(o, idx, std::sqrt(d2) * 3.048f);
    }
    API::get()->log_info("[Halo-CampE-UEVR] BLIPDUMP cap=%d done: %d neighbours within 36 m",
                         g_cfg.blip_dump, hits);
}

// ---- WRIST RADAR SCAN (doctrine at the exports in BlamDrive.hpp). Two cadences, per the
// vehfacing lesson (a table walk per publish shook the whole picture): the TABLE walk that finds
// unit-like objects runs every ~2500 calls (~0.3 Hz), caching pointers; the cheap position reads
// off the cache run every 32nd call. Everything sim-thread-local except the published atomics.
void blip_scan(uintptr_t player_obj) {
    // BlamDrive.cpp's own helpers, through the bridge: the same objects under the same names.
    const auto read_ptr = host::g_blamdrive_state.read_ptr;
    const uint32_t& g_tls_index = *host::g_blamdrive_state.tls_index;

    constexpr int MAX_TRACK = 24;
    static uintptr_t s_track[MAX_TRACK];
    static int       s_team[MAX_TRACK];
    static uint8_t   s_raw[MAX_TRACK];
    static uint32_t  s_type[MAX_TRACK];
    static float     s_px[MAX_TRACK], s_py[MAX_TRACK];   // last-sampled position (moving flag)
    static bool      s_moving[MAX_TRACK];
    // SPEED over WALL-CLOCK time, not displacement per N calls: the call cadence was never
    // measured, and the log caught the consequence red-handed -- contacts sprinting between
    // samples, every one flagged mv=0, no dots at all (and the old strobing dots were the same
    // broken window occasionally crossing threshold by luck). Two consecutive >=0.10 blam-unit/s
    // (~0.3 m/s) samples at >=130 ms spacing = a mover; single spikes stay filtered.
    static uint8_t   s_mvrun[MAX_TRACK];
    static long long s_mvat[MAX_TRACK];
    static int       s_ntrack = 0;
    static uint32_t  s_call = 0;
    ++s_call;

    if (IsBadReadPtr((const void*)(player_obj + 0x20), 12)) return;
    const float* pp = (const float*)(player_obj + 0x20);

    // REBUILD ON A WALL CLOCK, never on a call count. This was every 2500 calls, assumed to be
    // ~1 s from a sim rate that was never re-measured; it is actually ~125 ms, so the rebuild
    // wiped the movement window (130 ms) before it could ever close -- speeds measured fine at
    // 0.7-1.2 u/s while the run counter was reset to 0 forever and nothing ever counted as
    // moving. Probe-sampling aliasing, and the second time in this project.
    static long long s_scan_at = 0;
    const long long scan_now = now_ms();
    if (scan_now - s_scan_at >= 1000) {
        s_scan_at = scan_now;
        const uintptr_t tls_array = (uintptr_t)__readgsqword(0x58);
        uintptr_t block = 0, ctx = 0, table = 0;
        if (tls_array != 0 &&
            read_ptr(tls_array + (uintptr_t)g_tls_index * 8, &block) && block != 0 &&
            read_ptr(block + 0x20, &ctx) && ctx != 0 &&
            read_ptr(ctx + 0x50, &table) && table != 0) {
            // Snapshot the old set so a contact that survives the rebuild keeps its movement
            // history. Rebuilding into fresh state is what made the window unclosable.
            uintptr_t old_track[MAX_TRACK];
            float     old_px[MAX_TRACK], old_py[MAX_TRACK];
            bool      old_moving[MAX_TRACK];
            uint8_t   old_run[MAX_TRACK];
            long long old_at[MAX_TRACK];
            const int old_n = s_ntrack;
            for (int k = 0; k < old_n; ++k) {
                old_track[k] = s_track[k]; old_px[k] = s_px[k]; old_py[k] = s_py[k];
                old_moving[k] = s_moving[k]; old_run[k] = s_mvrun[k]; old_at[k] = s_mvat[k];
            }
            s_ntrack = 0;
            for (int idx = 0; idx < 4096 && s_ntrack < MAX_TRACK; ++idx) {
                uintptr_t o = 0;
                if (!read_ptr(table + (uintptr_t)idx * 24 + 0x10, &o) || o == 0) continue;
                if (o == player_obj || IsBadReadPtr((const void*)o, 0x180)) continue;
                // Unit-like: unattached, and the TEAM byte reads human or covenant.
                if (*(const uint32_t*)(o + 0x0C) != 0xFFFFFFFFu) continue;
                const uint8_t team = *(const uint8_t*)(o + 0x177);
                if (team != 0x0E && team != 0x0D) continue;
                const float* q = (const float*)(o + 0x20);
                const float ddx = q[0] - pp[0], ddy = q[1] - pp[1];
                if (!(ddx == ddx) || ddx * ddx + ddy * ddy > 20.0f * 20.0f) continue;
                s_track[s_ntrack] = o;
                s_team[s_ntrack] = (team == 0x0D) ? 1 : 0;
                s_raw[s_ntrack] = team;
                s_type[s_ntrack] = *(const uint32_t*)o;
                int prev = -1;
                for (int k = 0; k < old_n; ++k) if (old_track[k] == o) { prev = k; break; }
                if (prev >= 0) {
                    s_px[s_ntrack] = old_px[prev]; s_py[s_ntrack] = old_py[prev];
                    s_moving[s_ntrack] = old_moving[prev];
                    s_mvrun[s_ntrack] = old_run[prev];
                    s_mvat[s_ntrack] = old_at[prev];
                } else {
                    s_px[s_ntrack] = q[0]; s_py[s_ntrack] = q[1];
                    s_moving[s_ntrack] = false;
                    s_mvrun[s_ntrack] = 0;
                    s_mvat[s_ntrack] = scan_now;
                }
                ++s_ntrack;
            }
        }
    }

    // Positions publish every 8th call (the render-side jitter fix: ~4x the old rate); the
    // MOVING test stays on the original 32-call baseline, because its 2 cm threshold was sized
    // for the ~0.13 s window and a faster window can no longer see a slow walker over the noise.
    // FACTION-FIELD SURVEY (doctrine at Config::blip_bytes). Fires once per value change, on the
    // cached contact set, so both captures cover the same object shape.
    {
        static int s_bytes_armed = 0;
        if (g_cfg.blip_bytes != s_bytes_armed) {
            s_bytes_armed = g_cfg.blip_bytes;
            if (s_bytes_armed != 0) {
                for (int i = 0; i < s_ntrack; ++i) {
                    const uintptr_t o = s_track[i];
                    if (o == 0 || IsBadReadPtr((const void*)o, 0x200)) continue;
                    // TWO windows. The HEADER first: a Blam object's leading dwords carry its
                    // tag / definition index, which is a real SPECIES identity rather than the
                    // coarse body class at +0x177 (a marine and an Elite share that). If a
                    // header dword tracks species one-for-one, per-species colour keys on it.
                    char hdr[3 * 32 + 1];
                    int hw = 0;
                    for (int b = 0; b < 32; ++b)
                        hw += sprintf_s(hdr + hw, sizeof(hdr) - (size_t)hw, "%02X ",
                                        *(const uint8_t*)(o + b));
                    char buf[3 * 96 + 1];
                    int w = 0;
                    for (int b = 0; b < 96; ++b)
                        w += sprintf_s(buf + w, sizeof(buf) - (size_t)w, "%02X ",
                                       *(const uint8_t*)(o + 0x140 + b));
                    const float* q = (const float*)(o + 0x20);
                    const float ddx = q[0] - pp[0], ddy = q[1] - pp[1];
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] BLIPBYTES %d id=%08X cls=%02X d=%.1f +0x00: %s| +0x140: %s",
                        s_bytes_armed, (uint32_t)(o & 0xFFFFFFFFu), s_raw[i],
                        std::sqrt(ddx * ddx + ddy * ddy), hdr, buf);
                }
                API::get()->log_info("[Halo-CampE-UEVR] BLIPBYTES %d done: %d contacts",
                                     s_bytes_armed, s_ntrack);
            }
        }
    }

    if ((s_call % 8u) != 0u) return;
    int n = 0;
    for (int i = 0; i < s_ntrack && n < MAX_BLIPS; ++i) {
        const uintptr_t o = s_track[i];
        if (o == 0 || IsBadReadPtr((const void*)o, 0x180)) continue;
        if (*(const uint8_t*)(o + 0x177) != (s_team[i] == 1 ? 0x0D : 0x0E)) continue;  // slot recycled
        const float* q = (const float*)(o + 0x20);
        const float dx = q[0] - pp[0], dy = q[1] - pp[1];
        if (!(dx == dx) || dx * dx + dy * dy > 8.2f * 8.2f) continue;   // 25 m radar range
        // MOVING, the real tracker's rule: position changed since the last publish sample
        // (~0.13 s at this cadence; 2 cm noise floor).
        bool moving = s_moving[i];
        float dbg_speed = -1.0f;
        long long dbg_dtm = 0;
        {
            const long long nowm = now_ms();
            const long long dtm = nowm - s_mvat[i];
            dbg_dtm = dtm;
            if (dtm >= 130) {
                const float mx = q[0] - s_px[i], my = q[1] - s_py[i];
                const float speed = std::sqrt(mx * mx + my * my) * 1000.0f / (float)dtm;
                s_px[i] = q[0]; s_py[i] = q[1];
                s_mvat[i] = nowm;
                s_mvrun[i] = (speed > 0.10f) ? (uint8_t)((s_mvrun[i] < 250) ? s_mvrun[i] + 1 : 250) : 0;
                moving = s_mvrun[i] >= 2;   // sustained -- one twitch is a settle, not a contact
                s_moving[i] = moving;
                dbg_speed = speed;
            }
        }
        g_blip_dx[n].store(dx, std::memory_order_relaxed);
        g_blip_dy[n].store(dy, std::memory_order_relaxed);
        g_blip_team[n].store(s_team[i], std::memory_order_relaxed);
        g_blip_moving[n].store(moving, std::memory_order_relaxed);
        g_blip_id[n].store((uint32_t)(o & 0xFFFFFFFFu), std::memory_order_relaxed);
        g_blip_raw[n].store((uint32_t)s_raw[i], std::memory_order_relaxed);
        g_blip_type[n].store(s_type[i], std::memory_order_relaxed);
        g_blip_wx[n].store(q[0], std::memory_order_relaxed);
        g_blip_wy[n].store(q[1], std::memory_order_relaxed);
        g_blip_wz[n].store(q[2], std::memory_order_relaxed);
        if (dbg_speed >= 0.0f) g_blip_speed[n].store(dbg_speed, std::memory_order_relaxed);
        g_blip_dtm[n].store((int)dbg_dtm, std::memory_order_relaxed);
        g_blip_run[n].store((int)s_mvrun[i], std::memory_order_relaxed);
        ++n;
    }
    g_blip_count.store(n, std::memory_order_relaxed);
}

} // namespace

// ---- RADAR BLIPS, WRIST HUD / RADAR / TRACKER keys.
bool wristhud_parse_key(const char* key, const char* val, double v) {
    if (_stricmp(key, "blipdump")    == 0) { g_cfg.blip_dump      = (int)v; return true; }
    if (_stricmp(key, "blipcolor") == 0) {
        // <hex class>,<r>,<g>,<b>. Repeatable; a repeat of the same class overwrites it, so a
        // live edit retunes a colour instead of exhausting the table.
        unsigned cls = 0; float c[3] = {1.0f, 1.0f, 1.0f};
        if (sscanf_s(val, "%x,%f,%f,%f", &cls, &c[0], &c[1], &c[2]) >= 2) {
            int slot = -1;
            for (int i = 0; i < g_cfg.blip_color_n; ++i)
                if (g_cfg.blip_color[i].cls == cls) { slot = i; break; }
            if (slot < 0 && g_cfg.blip_color_n < kMaxBlipColor) slot = g_cfg.blip_color_n++;
            if (slot >= 0) {
                // Bump the generation ONLY on a real change. The file is re-read every ~2 s, so
                // bumping per parse threw away the colour cache and rebuilt every render target
                // continuously -- a standing leak dressed up as a feature.
                BlipColor& e = g_cfg.blip_color[slot];
                if (e.cls != (uint32_t)cls || e.r != c[0] || e.g != c[1] || e.b != c[2]) {
                    ++g_cfg.blip_color_gen;
                    e.cls = (uint32_t)cls;
                    e.r = c[0]; e.g = c[1]; e.b = c[2];
                }
            }
        }
        return true;
    }
    if (_stricmp(key, "blipcolorother") == 0) {
        float c[3] = {g_cfg.blip_color_other[0], g_cfg.blip_color_other[1], g_cfg.blip_color_other[2]};
        sscanf_s(val, "%f,%f,%f", &c[0], &c[1], &c[2]);
        if (c[0] != g_cfg.blip_color_other[0] || c[1] != g_cfg.blip_color_other[1] ||
            c[2] != g_cfg.blip_color_other[2]) {
            ++g_cfg.blip_color_gen;
            g_cfg.blip_color_other[0] = c[0];
            g_cfg.blip_color_other[1] = c[1];
            g_cfg.blip_color_other[2] = c[2];
        }
        return true;
    }
    if (_stricmp(key, "blipname") == 0) {
        char m[64] = {0}; float c[3] = {1.0f, 1.0f, 1.0f};
        if (sscanf_s(val, "%63[^,],%f,%f,%f", m, (unsigned)sizeof(m), &c[0], &c[1], &c[2]) >= 2) {
            int slot = -1;
            for (int i = 0; i < g_cfg.blip_name_n; ++i)
                if (_stricmp(g_cfg.blip_name[i].match, m) == 0) { slot = i; break; }
            if (slot < 0 && g_cfg.blip_name_n < kMaxBlipName) slot = g_cfg.blip_name_n++;
            if (slot >= 0) {
                BlipName& e = g_cfg.blip_name[slot];
                if (_stricmp(e.match, m) != 0 || e.r != c[0] || e.g != c[1] || e.b != c[2]) {
                    ++g_cfg.blip_color_gen;
                    strncpy_s(e.match, m, _TRUNCATE);
                    e.r = c[0]; e.g = c[1]; e.b = c[2];
                }
            }
        }
        return true;
    }
    if (_stricmp(key, "blipbytes")   == 0) { g_cfg.blip_bytes     = (int)v; return true; }
    if (_stricmp(key, "trackerdump") == 0) { g_cfg.tracker_dump   = (int)v; return true; }
    if (_stricmp(key, "trackermid")  == 0) { g_cfg.tracker_mid    = (int)v; return true; }
    if (_stricmp(key, "wristradar")  == 0) { g_cfg.wrist_radar    = (v != 0.0); return true; }
    if (_stricmp(key, "wristradarblip") == 0) { g_cfg.wrist_radar_blip = clampf((float)v, 0.002f, 0.1f); return true; }
    if (_stricmp(key, "wristradargain") == 0) { g_cfg.wrist_radar_gain = clampf((float)v, 1.0f, 100000.0f); return true; }
    if (_stricmp(key, "wristradaraimsign") == 0) { g_cfg.wrist_radar_aimsign = (v < 0.0) ? -1.0f : 1.0f; return true; }
    if (_stricmp(key, "wristradarrot")  == 0) { g_cfg.wrist_radar_rot  = (float)v; return true; }
    if (_stricmp(key, "wristradarflip") == 0) { g_cfg.wrist_radar_flip = (v != 0.0) ? 1 : 0; return true; }
    if (_stricmp(key, "wristradarlog")  == 0) { g_cfg.wrist_radar_log  = (v != 0.0); return true; }
    if (_stricmp(key, "wristradartest") == 0) { g_cfg.wrist_radar_test = (int)v; return true; }
    if (_stricmp(key, "wristradarcenter") == 0) { sscanf_s(val, "%f,%f", &g_cfg.wrist_radar_center[0], &g_cfg.wrist_radar_center[1]); return true; }
    if (_stricmp(key, "wristradartilt") == 0) { g_cfg.wrist_radar_tilt = clampf((float)v, -90.0f, 90.0f); return true; }
    if (_stricmp(key, "wristhudblend") == 0) { g_cfg.wrist_hud_blend = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "wristhudblendr") == 0) { g_cfg.wrist_hud_blend_r = (int)clampf((float)v, 0.0f, 2.0f); return true; }
    if (_stricmp(key, "wristhudgainr") == 0) { g_cfg.wrist_hud_gain_r = clampf((float)v, 0.05f, 100.0f); return true; }
    if (_stricmp(key, "wristhud")        == 0) { g_cfg.wrist_hud = (v != 0.0); return true; }
    if (_stricmp(key, "wristhudclasses") == 0) { strncpy_s(g_cfg.wrist_hud_classes, val, _TRUNCATE); return true; }
    if (_stricmp(key, "wristhudoff")     == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.wrist_hud_off[0], &g_cfg.wrist_hud_off[1], &g_cfg.wrist_hud_off[2]); return true; }
    if (_stricmp(key, "wristhudrot")     == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.wrist_hud_rot[0], &g_cfg.wrist_hud_rot[1], &g_cfg.wrist_hud_rot[2]); return true; }
    if (_stricmp(key, "wristhudscale")   == 0) { g_cfg.wrist_hud_scale = clampf((float)v, 0.005f, 1.0f); return true; }
    if (_stricmp(key, "wristhudgap")     == 0) { g_cfg.wrist_hud_gap = clampf((float)v, 0.0f, 0.5f); return true; }
    if (_stricmp(key, "wristhuddraw")    == 0) { g_cfg.wrist_hud_draw = clampf((float)v, 64.0f, 2048.0f); return true; }
    if (_stricmp(key, "wristhudtrigger") == 0) { g_cfg.wrist_hud_trigger = (v != 0.0); return true; }
    if (_stricmp(key, "wristhudclassesr") == 0) { strncpy_s(g_cfg.wrist_hud_classes_r, val, _TRUNCATE); return true; }
    if (_stricmp(key, "wristhudoffr")    == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.wrist_hud_off_r[0], &g_cfg.wrist_hud_off_r[1], &g_cfg.wrist_hud_off_r[2]); return true; }
    if (_stricmp(key, "wristhudrotr")    == 0) { sscanf_s(val, "%f,%f,%f", &g_cfg.wrist_hud_rot_r[0], &g_cfg.wrist_hud_rot_r[1], &g_cfg.wrist_hud_rot_r[2]); return true; }
    if (_stricmp(key, "wristhudgapr")    == 0) { g_cfg.wrist_hud_gap_r = clampf((float)v, 0.0f, 0.5f); return true; }
    if (_stricmp(key, "hudplacement")   == 0) { g_cfg.hud_placement = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "hudwpnanchor")   == 0) { g_cfg.hud_wpn_anchor = (int)clampf((float)v, 1.0f, 3.0f); return true; }
    if (_stricmp(key, "hudwpntracker")  == 0) { float* s = g_cfg.hud_wpn_tracker; sscanf_s(val, "%f,%f,%f,%f,%f,%f", &s[0], &s[1], &s[2], &s[3], &s[4], &s[5]); return true; }
    if (_stricmp(key, "hudwpnshield")   == 0) { float* s = g_cfg.hud_wpn_shield;  sscanf_s(val, "%f,%f,%f,%f,%f,%f", &s[0], &s[1], &s[2], &s[3], &s[4], &s[5]); return true; }
    if (_stricmp(key, "hudwpnammo")     == 0) { float* s = g_cfg.hud_wpn_ammo;    sscanf_s(val, "%f,%f,%f,%f,%f,%f", &s[0], &s[1], &s[2], &s[3], &s[4], &s[5]); return true; }
    if (_stricmp(key, "hudwpngrenade")  == 0) { float* s = g_cfg.hud_wpn_grenade; sscanf_s(val, "%f,%f,%f,%f,%f,%f", &s[0], &s[1], &s[2], &s[3], &s[4], &s[5]); return true; }
    if (_stricmp(key, "hudwpngap")      == 0) { g_cfg.hud_wpn_gap = clampf((float)v, 0.0f, 50.0f); return true; }
    if (_stricmp(key, "hudwpnscale")    == 0) { g_cfg.hud_wpn_scale = clampf((float)v, 0.002f, 0.2f); return true; }
    if (_stricmp(key, "hudwpnfallback") == 0) { g_cfg.hud_wpn_fallback = (int)clampf((float)v, 0.0f, 1.0f); return true; }
    if (_stricmp(key, "hudwpnlog")      == 0) { g_cfg.hud_wpn_log = (v != 0.0); return true; }
    if (_stricmp(key, "wristhudammotext") == 0) { strncpy_s(g_cfg.wrist_hud_ammo_text, val, _TRUNCATE); return true; }
    return false;
}

namespace {

void wristhud_game_tick_late() {
    g_tick_stage = "wristhud";
    wristhud_tick();   // wrist HUD: census + hosting + forearm placement
}

// WRIST HUD PLACEMENT, in the render pass rather than on the tick: the camera published just before it
// is the one this frame is drawn from, so the forearm panels land against it instead of against a
// camera several milliseconds stale. Once per frame, not per eye.
void wristhud_render_frame() {
    halo::wristhud_place();
}

void wristhud_xinput_after_calib_trigger(_XINPUT_STATE* state) {
        // ---- WRIST HUD GLANCE GATE. After the calibration eat above so calibration keeps
        // precedence (its zeroed trigger reads as "not held" here). While the trigger serves the
        // HUD it is swallowed from the game -- a glance must not also fire LT's native action.
        // NOT in stick mode: the panels are hidden in vehicles and cutscenes anyway, so eating the
        // trigger there is pure loss -- and LT is the vehicle handbrake / quick turn, so the
        // glance gate was silently disabling a driving control for no benefit.
        if (g_cfg.enabled && g_cfg.wrist_hud && g_cfg.wrist_hud_trigger &&
            !halo::g_stick_mode_active.load(std::memory_order_relaxed)) {
            g_wristhud_lt.store(state->Gamepad.bLeftTrigger >= 64, std::memory_order_relaxed);
            state->Gamepad.bLeftTrigger = 0;
        } else {
            g_wristhud_lt.store(false, std::memory_order_relaxed);
        }
}

void wristhud_sim_unit_state_radar(uintptr_t obj) {
    blip_dump_probe(obj);
    if (g_cfg.wrist_hud && g_cfg.wrist_radar) blip_scan(obj);   // the radar exists only with the wrist HUD
}

float wristhud_widget_tint_mul() {
    return tl_tint_mul;
}

} // namespace

constinit const FeatureHooks kWristHudHooks{
    .key                        = "wristhud",
    .parse_key                  = &wristhud_parse_key,
    .game_tick_late             = &wristhud_game_tick_late,
    .render_frame               = &wristhud_render_frame,
    .xinput_after_calib_trigger = &wristhud_xinput_after_calib_trigger,
    .sim_unit_state_radar       = &wristhud_sim_unit_state_radar,
    .widget_tint_mul            = &wristhud_widget_tint_mul,
};

} // namespace halo
