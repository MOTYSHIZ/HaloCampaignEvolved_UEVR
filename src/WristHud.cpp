#include "WristHud.hpp"

#include "BlamDrive.hpp"          // the wrist radar's published blips
#include "Config.hpp"
#include "Holster.hpp"            // grenade meshes as blip art
#include "Markers.hpp"            // holster_marker_place_rot / scale / room_to_world
#include "Math.hpp"
#include "Rig.hpp"              // call_ret_vec3
#include "MotionAimControl.hpp"   // get_pose, g_stick_mode_active
#include "Reticule.hpp"           // widget_quad_begin/finish -- THE one copy of the quad recipe
#include "UeObject.hpp"
#include "uevr/API.hpp"

#include <cmath>
#include <cstring>
#include <string>
#include <functional>
#include <vector>

using uevr::API;

namespace halo {

extern std::atomic<float> g_view_base_yaw;   // Plugin.cpp: the yaw handed to UEVR every frame

std::atomic<bool> g_wristhud_lt{false};
std::atomic<bool> g_wristhud_hide_cradle{false};

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

// DISCOVERY ONLY, once per game tick: census, hosting, and the per-tick colour chain. Placement
// deliberately does NOT live here -- see wristhud_place().
void wristhud_tick() {
    ++s_tick;
    tracker_dump_probe();
    tracker_mid_probe(s_tick);
    if (!g_cfg.wrist_hud) return;
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
        apply_widget_tint_scaled(comp, s_slots[si].right ? g_cfg.wrist_hud_gain_r : 1.0f, false);
        const bool right = s_slots[si].right;
        const bool have = right ? have_r : have_l;
        const int  sidx = right ? idx_r++ : idx_l++;
        if (!have) { holster_marker_show(comp, false); continue; }
        holster_marker_show(comp, true);
        if (s_slots[si].match.find(L"WeaponCradle") != std::wstring::npos) {
            if (g_wristhud_hide_cradle.load(std::memory_order_relaxed)) wh_cradle_write_zero(s_slots[si].widget.get());
            else wh_cradle_restore();
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
        auto* mf = holster_mesh_frag();
        auto* mp = holster_mesh_plasma();
        const float half = g_cfg.wrist_hud_draw * g_cfg.wrist_hud_scale / 200.0f;   // panel half-width, room metres
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
            const double dscale = (double)g_cfg.wrist_radar_blip * 2.5;
            const double mscale = (double)g_cfg.wrist_radar_blip * 12.5;
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

} // namespace halo
