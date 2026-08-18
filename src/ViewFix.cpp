#include "ViewFix.hpp"

#include "Config.hpp"
#include "UeObject.hpp"
#include "uevr/API.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>

using namespace uevr;

namespace halo {
namespace {

// Parameter offsets resolved from reflection, never hand-summed -- the HitTrace doctrine. A
// wrong hardcoded layout here would not crash; it would write the rotation into padding and
// present as "audiofix does nothing", which is indistinguishable from the Wwise-ignores-it
// outcome this feature exists to test. Resolving removes that ambiguity.
std::wstring vf_field_name(API::FField* f) {
    if (f == nullptr) return L"";
    auto* n = f->get_fname();
    return (n != nullptr) ? n->to_string() : L"";
}

int32_t vf_offset_of(API::UStruct* s, const wchar_t* want) {
    if (s == nullptr) return -1;
    for (auto* f = s->get_child_properties(); f != nullptr; f = f->get_next()) {
        if (vf_field_name(f) == want) {
            return reinterpret_cast<API::FProperty*>(f)->get_offset();
        }
    }
    return -1;
}

// SetAudioListenerOverride / ClearAudioListenerOverride, resolved once per PlayerController
// CLASS and re-resolved when the class changes (level transitions can swap controller classes;
// the frontend controller is a different class entirely). UFunction objects live on the class,
// not the instance, so caching against the class is recycle-safe where caching against the
// instance would not be.
API::UFunction* g_fn_set   = nullptr;
API::UFunction* g_fn_clear = nullptr;
void*           g_fn_class = nullptr;

int32_t g_off_attach = -1;
int32_t g_off_loc    = -1;
int32_t g_off_rot    = -1;
bool    g_lwc_double = true;    // derived from the Location->Rotation gap, not assumed

bool g_audio_applied        = false;   // an override is currently in place and must be cleared
bool g_audio_missing_logged = false;   // "not found" said once per class, not per tick

bool resolve_listener_functions(API::UObject* pc) {
    auto* cls = pc->get_class();
    if (cls == nullptr) return false;
    if ((void*)cls != g_fn_class) {
        g_fn_class = (void*)cls;
        g_fn_set   = cls->find_function(L"SetAudioListenerOverride");
        g_fn_clear = cls->find_function(L"ClearAudioListenerOverride");
        g_audio_missing_logged = false;
        g_off_attach = vf_offset_of(g_fn_set, L"AttachToComponent");
        g_off_loc    = vf_offset_of(g_fn_set, L"Location");
        g_off_rot    = vf_offset_of(g_fn_set, L"Rotation");
        // FVector is 24 bytes under LWC doubles, 12 as floats: the gap between the two vector
        // params says which this build uses, so a float build mis-sizes nothing.
        g_lwc_double = (g_off_loc >= 0 && g_off_rot >= 0) ? ((g_off_rot - g_off_loc) >= 24) : true;
        if (g_fn_set != nullptr && g_fn_clear != nullptr) {
            API::get()->log_info("[Halo-CampE-UEVR] AUDIOFIX: listener override resolved "
                                 "(attach@%d loc@%d rot@%d %s, params=%d)",
                                 g_off_attach, g_off_loc, g_off_rot,
                                 g_lwc_double ? "double" : "float",
                                 g_fn_set->get_properties_size());
        }
    }
    if (g_fn_set == nullptr || g_fn_clear == nullptr
        || g_off_attach < 0 || g_off_loc < 0 || g_off_rot < 0) {
        if (!g_audio_missing_logged) {
            g_audio_missing_logged = true;
            // This is a FINDING, not just a failure: it means the engine-side listener lane does
            // not exist on this build and the Wwise AkComponent lane is the one to pursue.
            API::get()->log_info("[Halo-CampE-UEVR] AUDIOFIX: SetAudioListenerOverride unusable on "
                                 "this PlayerController class (set=%p clear=%p attach@%d loc@%d "
                                 "rot@%d) -- audiofix is inert on this build; next lane is "
                                 "audiodump=1 to survey the Wwise listener objects",
                                 (void*)g_fn_set, (void*)g_fn_clear,
                                 g_off_attach, g_off_loc, g_off_rot);
        }
        return false;
    }
    return true;
}

}  // namespace

void audio_fix_tick(bool engaged,
                    float view_x, float view_y, float view_z,
                    float view_yaw_deg, float view_pitch_deg) {
    const bool want = engaged && g_cfg.audio_fix;

    auto* pc = API::get()->get_player_controller(0);

    if (!want) {
        // RELEASE, once. If the controller is gone the override died with it -- nothing to clear.
        if (g_audio_applied) {
            g_audio_applied = false;
            if (pc != nullptr && resolve_listener_functions(pc)) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                pc->process_event(g_fn_clear, p);
            }
            API::get()->log_info("[Halo-CampE-UEVR] AUDIOFIX: listener override cleared");
        }
        return;
    }

    if (pc == nullptr) return;
    if (!resolve_listener_functions(pc)) return;
    if (!std::isfinite(view_x) || !std::isfinite(view_yaw_deg) || !std::isfinite(view_pitch_deg)) return;

    // Re-sent EVERY tick, not on-change: the transform changes every frame by nature (this is a
    // per-frame drive like the rig write, not a tunable), and re-sending also self-heals across
    // PlayerController swaps -- a recycled controller simply gets the override again next tick.
    // Position is the game camera's (published pre-HMD-offset); rotation is the composed rendered
    // view. Rotation is the term that matters for panning -- the README's own error law is "drifts
    // by however far your head is turned off the game's view" -- position error is centimetres.
    alignas(16) uint8_t buf[RIG_PARAM_BUF] = {0};
    *reinterpret_cast<void**>(buf + g_off_attach) = nullptr;   // world-fixed, re-sent per tick
    if (g_lwc_double) {
        auto* loc = reinterpret_cast<double*>(buf + g_off_loc);
        loc[0] = (double)view_x; loc[1] = (double)view_y; loc[2] = (double)view_z;
        auto* rot = reinterpret_cast<double*>(buf + g_off_rot);
        rot[0] = (double)view_pitch_deg;   // FRotator order: Pitch, Yaw, Roll
        rot[1] = (double)view_yaw_deg;
        rot[2] = 0.0;                      // roll: not published, and inaudible next to yaw
    } else {
        auto* loc = reinterpret_cast<float*>(buf + g_off_loc);
        loc[0] = view_x; loc[1] = view_y; loc[2] = view_z;
        auto* rot = reinterpret_cast<float*>(buf + g_off_rot);
        rot[0] = view_pitch_deg;
        rot[1] = view_yaw_deg;
        rot[2] = 0.0f;
    }
    pc->process_event(g_fn_set, buf);

    if (!g_audio_applied) {
        g_audio_applied = true;
        API::get()->log_info("[Halo-CampE-UEVR] AUDIOFIX: listener override ENGAGED "
                             "(following rendered view; clear with audiofix=0)");
    }
}

// ---- THE CULLING FIX ---------------------------------------------------------------------------

void cull_fix_tick() {
    static bool  applied      = false;
    static bool  applied_head = false;
    static float applied_dist = -1.0f;
    static void* applied_pc   = nullptr;
    static uint32_t polls_since = 0;

    // TWO INDEPENDENT LEVERS, deliberately: cull_fix owns the distance override, cull_head owns
    // the head-keyed relevancy terms. Decoupled (2026-08-15, user-designed test) because the
    // clean endpoint is cull_head ALONE against the game's STOCK distance -- if the head-keyed
    // terms hold, the override is not merely reducible but unnecessary, and any residual
    // override is pure perf divergence. Since exec'd cvars have no un-exec, the no-override
    // state requires cullfix=0 FROM BOOT, which this gating makes expressible.
    if (!g_cfg.cull_fix && !g_cfg.cull_head) {
        if (applied) {
            applied = false;
            applied_pc = nullptr;
            // No un-exec exists for a cvar whose stock value is unreadable on this build --
            // say so instead of pretending the toggle restored anything.
            API::get()->log_info("[Halo-CampE-UEVR] CULLFIX: no longer re-applied (both keys 0); "
                                 "restart the game to restore stock relevancy behaviour");
        }
        return;
    }

    auto* pc = API::get()->get_player_controller(0);
    if (pc == nullptr) { applied_pc = nullptr; return; }   // between levels: wait

    ++polls_since;
    const bool changed = !applied || applied_dist != g_cfg.cull_dist || applied_pc != (void*)pc
                      || applied_head != g_cfg.cull_head;
    // The timer term is insurance: a level load can reset the cvar AND recycle the controller
    // to the same address, which the pointer compare cannot see. The exec is idempotent and
    // costs nothing at this cadence (~1/min at the ~2 s poll), so re-applying silently is
    // cheaper than any detection that would need a cvar READ -- reads are garbage on this build.
    if (!changed && polls_since < 32) return;
    polls_since = 0;

    if (g_cfg.cull_fix) {
        wchar_t cmd[128] = {0};
        swprintf_s(cmd, L"Blam.Synchronization.Relevancy.OutOfViewCullDistance %.0f",
                   (double)g_cfg.cull_dist);
        API::get()->execute_command(cmd);
    }

    // HEAD-KEYED RELEVANCY (cullhead): shift the relevancy system's visibility inputs onto the
    // UE-side terms, which are computed from the RENDERED view -- the player's head -- instead
    // of Blam's aim camera. If these hold, creatures wake when LOOKED at (not aimed at) and the
    // cull follows the head, making the game's STOCK cull distance correct again (things you
    // are not looking at may cull at the tuned distance -- you are not looking). Semantics are
    // unverified on this build (the CullByBlamVisibility lesson: relevancy flags gate wake AND
    // cull), so this ships default-off behind its own key and the in-headset ladder owns the
    // verdict. No un-exec exists; key off + restart restores stock.
    if (g_cfg.cull_head) {
        API::get()->execute_command(L"Blam.Synchronization.Relevancy.ByUEVisibility 1");
        API::get()->execute_command(L"Blam.Synchronization.Relevancy.CheckUELastRenderedTime 1");
    }
    if (changed) {
        API::get()->log_info("[Halo-CampE-UEVR] CULLFIX: applied%s%s "
                             "(re-applied per level and ~1/min; keys 0 + restart reverts)",
                             g_cfg.cull_fix ? " distance-override" : " (stock distance)",
                             g_cfg.cull_head ? " + HEAD-KEYED relevancy (ByUEVisibility, "
                                               "CheckUELastRenderedTime)" : "");
    }
    applied = true;
    applied_dist = g_cfg.cull_dist;
    applied_head = g_cfg.cull_head;
    applied_pc = (void*)pc;
}

// ---- LANE 2: the game's own listener component -------------------------------------------------

namespace {

// The live listener, held recycle-safe. Resolved by class-name sweep because the owning actor is
// a level object (BP_BlamCameraManager_C) that is recreated per level -- a cached pointer would
// pin residue, the standard failure mode on this title.
TrackedObject g_ak_listener;
uint32_t      g_ak_scan_tick = 0;

// K2_SetWorldLocationAndRotation / K2_SetRelativeLocationAndRotation, offsets reflection-derived
// once per component class (same doctrine as the override above).
API::UFunction* g_ak_set_world = nullptr;
API::UFunction* g_ak_set_rel   = nullptr;
void*           g_ak_cls       = nullptr;
int32_t g_akw_loc = -1, g_akw_rot = -1, g_akw_sweep = -1, g_akw_tele = -1;
int32_t g_akr_loc = -1, g_akr_rot = -1, g_akr_sweep = -1, g_akr_tele = -1;

// The component's authored relative transform, captured at resolve so release can put it back.
// Raw property reads (LWC doubles); identity fallback if the read fails.
double g_ak_rel_loc[3] = {0, 0, 0};
double g_ak_rel_rot[3] = {0, 0, 0};
bool   g_ak_applied = false;

bool ak_resolve_functions(API::UObject* comp) {
    auto* cls = comp->get_class();
    if (cls == nullptr) return false;
    if ((void*)cls != g_ak_cls) {
        g_ak_cls = (void*)cls;
        g_ak_set_world = cls->find_function(L"K2_SetWorldLocationAndRotation");
        g_ak_set_rel   = cls->find_function(L"K2_SetRelativeLocationAndRotation");
        g_akw_loc   = vf_offset_of(g_ak_set_world, L"NewLocation");
        g_akw_rot   = vf_offset_of(g_ak_set_world, L"NewRotation");
        g_akw_sweep = vf_offset_of(g_ak_set_world, L"bSweep");
        g_akw_tele  = vf_offset_of(g_ak_set_world, L"bTeleport");
        g_akr_loc   = vf_offset_of(g_ak_set_rel, L"NewLocation");
        g_akr_rot   = vf_offset_of(g_ak_set_rel, L"NewRotation");
        g_akr_sweep = vf_offset_of(g_ak_set_rel, L"bSweep");
        g_akr_tele  = vf_offset_of(g_ak_set_rel, L"bTeleport");
        API::get()->log_info("[Halo-CampE-UEVR] AUDIOCOMP: setters resolved "
                             "(world=%p loc@%d rot@%d | rel=%p loc@%d rot@%d)",
                             (void*)g_ak_set_world, g_akw_loc, g_akw_rot,
                             (void*)g_ak_set_rel, g_akr_loc, g_akr_rot);
    }
    return g_ak_set_world != nullptr && g_akw_loc >= 0 && g_akw_rot >= 0;
}

// Fill and fire one of the two setters. bSweep=false, bTeleport=true -- an audio listener has no
// collision story, and teleport skips the sweep machinery entirely.
void ak_call_setter(API::UObject* comp, API::UFunction* fn,
                    int32_t off_loc, int32_t off_rot, int32_t off_sweep, int32_t off_tele,
                    const double loc[3], const double rot[3]) {
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    auto* l = reinterpret_cast<double*>(p + off_loc);
    l[0] = loc[0]; l[1] = loc[1]; l[2] = loc[2];
    auto* r = reinterpret_cast<double*>(p + off_rot);
    r[0] = rot[0]; r[1] = rot[1]; r[2] = rot[2];   // Pitch, Yaw, Roll
    if (off_sweep >= 0) p[off_sweep] = 0;
    if (off_tele >= 0)  p[off_tele]  = 1;
    comp->process_event(fn, p);
}

}  // namespace

void audio_comp_tick(bool engaged, uint32_t tick,
                     float view_x, float view_y, float view_z,
                     float view_yaw_deg, float view_pitch_deg) {
    const bool want = engaged && g_cfg.audio_comp;

    if (!want) {
        // RELEASE: put the authored relative transform back, once, so the listener rides the
        // camera again exactly as shipped -- in stick mode that IS the correct listener.
        if (g_ak_applied) {
            g_ak_applied = false;
            auto* comp = g_ak_listener.get_checked(L"HaloAudioListener");
            if (comp != nullptr && g_ak_set_rel != nullptr && g_akr_loc >= 0 && g_akr_rot >= 0) {
                ak_call_setter(comp, g_ak_set_rel, g_akr_loc, g_akr_rot, g_akr_sweep, g_akr_tele,
                               g_ak_rel_loc, g_ak_rel_rot);
            }
            API::get()->log_info("[Halo-CampE-UEVR] AUDIOCOMP: listener returned to the camera");
        }
        return;
    }

    auto* comp = g_ak_listener.get_checked(L"HaloAudioListener");
    if (comp == nullptr) {
        // Sweep for the live instance, throttled exactly like the reticle scan -- and only while
        // unresolved, so the steady state costs one tracked-pointer check per tick.
        //
        // BUT A THROTTLE ALONE IS NOT A TERMINATION CONDITION. If HaloAudioListener never resolves
        // -- a rename in a game patch, or a mode that has no listener -- "only while unresolved"
        // means forever, and this becomes a full ~296k sweep every ~3.75 s for the whole session.
        // So failure is counted and eventually gives up: losing the audio fix is a far better
        // outcome than a permanent periodic stall, and the log says which happened.
        constexpr int kMaxFailures = 12;
        static int s_failures = 0;
        if (s_failures >= kMaxFailures) return;

        if (tick - g_ak_scan_tick < 120) return;
        g_ak_scan_tick = tick;
        auto* arr = API::get()->get_uobject_array();
        if (arr == nullptr) return;

        // Memoised per sweep, for the reason given at the reticle_rescan and resolve_rig sweeps:
        // class_name_of returns a wstring by value, so the bare form is ~296k allocations here.
        std::unordered_map<const void*, std::wstring> name_of_class;

        const int32_t n = arr->get_object_count();
        for (int32_t i = 0; i < n; ++i) {
            auto* o = arr->get_object(i);
            if (o == nullptr) continue;
            auto* cls = o->get_class();
            if (cls == nullptr) continue;
            auto memo = name_of_class.find(cls);
            if (memo == name_of_class.end()) memo = name_of_class.emplace(cls, class_name_of(o)).first;
            const std::wstring& cn = memo->second;
            if (cn.find(L"HaloAudioListener") == std::wstring::npos) continue;
            if (o == cls->get_class_default_object()) continue;
            g_ak_listener.set_at(o, i);
            comp = o;
            // Capture the authored relative transform for the release path. Raw property reads;
            // on this title reflection property access works for engine-side components (the rig
            // reads AttachChildren the same way). Identity stays if either read fails.
            if (auto* rl = o->get_property_data<double>(L"RelativeLocation")) {
                g_ak_rel_loc[0] = rl[0]; g_ak_rel_loc[1] = rl[1]; g_ak_rel_loc[2] = rl[2];
            }
            if (auto* rr = o->get_property_data<double>(L"RelativeRotation")) {
                g_ak_rel_rot[0] = rr[0]; g_ak_rel_rot[1] = rr[1]; g_ak_rel_rot[2] = rr[2];
            }
            API::get()->log_info("[Halo-CampE-UEVR] AUDIOCOMP: listener resolved (%s) "
                                 "rel=(%.1f,%.1f,%.1f | %.1f,%.1f,%.1f)",
                                 narrow(cn).c_str(),
                                 g_ak_rel_loc[0], g_ak_rel_loc[1], g_ak_rel_loc[2],
                                 g_ak_rel_rot[0], g_ak_rel_rot[1], g_ak_rel_rot[2]);
            break;
        }
        if (comp == nullptr) {
            if (++s_failures >= kMaxFailures) {
                API::get()->log_info("[Halo-CampE-UEVR] AUDIOCOMP: no HaloAudioListener after %d "
                                     "sweeps -- giving up. Positional audio keeps following the "
                                     "game camera rather than your head for this session.",
                                     kMaxFailures);
            }
            return;
        }
        s_failures = 0;
    }

    if (!ak_resolve_functions(comp)) return;
    if (!std::isfinite(view_x) || !std::isfinite(view_yaw_deg) || !std::isfinite(view_pitch_deg)) return;

    // WORLD write every tick, same shape as the rig: the parent (the Blam camera) is re-stamped
    // by the game continuously, so a relative cancellation computed at tick rate would be stale
    // between ticks. A world write is correct at the instant it lands; at ~32 Hz the residual is
    // bounded by how far the aim camera moves in ~31 ms, which the ear does not resolve.
    const double loc[3] = { (double)view_x, (double)view_y, (double)view_z };
    const double rot[3] = { (double)view_pitch_deg, (double)view_yaw_deg, 0.0 };
    ak_call_setter(comp, g_ak_set_world, g_akw_loc, g_akw_rot, g_akw_sweep, g_akw_tele, loc, rot);

    if (!g_ak_applied) {
        g_ak_applied = true;
        API::get()->log_info("[Halo-CampE-UEVR] AUDIOCOMP: listener DRIVEN to rendered view "
                             "(clear with audiocomp=0)");
    }
}

#if HALO_VR_DEV

void dev_exec_tick() {
    static char last[4][sizeof(g_cfg.dev_exec[0])] = {};
    for (int i = 0; i < 4; ++i) {
        const char* cmd = g_cfg.dev_exec[i];
        if (strcmp(cmd, last[i]) == 0) continue;    // on-change only -- never re-exec per poll
        strcpy_s(last[i], sizeof(last[i]), cmd);
        if (cmd[0] == '\0') {
            API::get()->log_info("[Halo-CampE-UEVR] DEVEXEC[%d]: cleared -- note there is no "
                                 "un-exec; set the inverse command or restart to revert", i + 1);
            continue;
        }
        wchar_t w[sizeof(g_cfg.dev_exec[0])] = {0};
        for (size_t k = 0; cmd[k] != '\0' && k < (sizeof(w) / sizeof(w[0])) - 1; ++k) {
            w[k] = (wchar_t)(unsigned char)cmd[k];
        }
        API::get()->execute_command(w);
        // Loud on purpose, same doctrine as DEV OVERRIDES ACTIVE: an experiment must not be
        // mistakable for shipping behaviour when a log comes back attached to a bug report.
        API::get()->log_info("[Halo-CampE-UEVR] DEVEXEC[%d] ACTIVE: %s", i + 1, cmd);
    }
}

void audio_dump_tick() {
    static int last = 0;
    if (g_cfg.audio_dump == last) return;           // edge-triggered: bump the value to re-run
    last = g_cfg.audio_dump;
    if (g_cfg.audio_dump == 0) return;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;

    API::get()->log_info("[Halo-CampE-UEVR] AUDIODUMP: surveying object array for "
                         "Ak/Audio/Listener/Wwise classes...");
    int logged = 0;
    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n && logged < 200; ++i) {
        auto* o = arr->get_object(i);
        if (o == nullptr) continue;
        const std::wstring cn = class_name_of(o);
        if (cn.empty()) continue;
        const bool hit = cn.find(L"AkComponent") != std::wstring::npos
                      || cn.find(L"AkGameObject") != std::wstring::npos
                      || cn.find(L"Listener") != std::wstring::npos
                      || cn.find(L"Wwise") != std::wstring::npos
                      || cn.find(L"AudioDevice") != std::wstring::npos;
        if (!hit) continue;
        auto* cls = o->get_class();
        if (cls != nullptr && o == cls->get_class_default_object()) continue;   // instances only

        // Outer chain, so "which actor owns the listener" is answerable from the log alone --
        // the whole point of the survey is finding the component that RIDES THE CAMERA.
        std::string path;
        for (API::UObject* p = o; p != nullptr; p = p->get_outer()) {
            const auto* fn = p->get_fname();
            path = (fn != nullptr ? narrow(fn->to_string()) : std::string("?")) +
                   (path.empty() ? "" : "." + path);
        }
        API::get()->log_info("[Halo-CampE-UEVR] AUDIODUMP: %s  |  %s", narrow(cn).c_str(), path.c_str());
        ++logged;
    }
    API::get()->log_info("[Halo-CampE-UEVR] AUDIODUMP: done (%d object(s) listed)", logged);
}

namespace {

// One class's reflected property surface, a few supers deep. Names + offsets only -- the point
// is finding the camera-rotation INPUT the CHUD projection consumes, and a name like
// "OnCameraRotationChanged" or "CameraRotation" in this list is the finding.
void nav_dump_struct_chain(API::UStruct* s, int supers) {
    for (API::UStruct* c = s; c != nullptr && supers > 0; c = c->get_super_struct(), --supers) {
        const auto* cn = c->get_fname();
        API::get()->log_info("[Halo-CampE-UEVR]   == %s (props 0x%X)",
                             cn != nullptr ? narrow(cn->to_string()).c_str() : "?",
                             c->get_properties_size());
        int n = 0;
        for (auto* f = c->get_child_properties(); f != nullptr && n < 96; f = f->get_next(), ++n) {
            const std::wstring fn = vf_field_name(f);
            if (fn.empty()) continue;
            API::get()->log_info("[Halo-CampE-UEVR]     +0x%04X %s",
                                 reinterpret_cast<API::FProperty*>(f)->get_offset(),
                                 narrow(fn).c_str());
        }
    }
}

}  // namespace

void nav_dump_tick() {
    static int last = 0;
    if (g_cfg.nav_dump == last) return;             // edge-triggered: bump the value to re-run
    last = g_cfg.nav_dump;
    if (g_cfg.nav_dump == 0) return;

    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;

    API::get()->log_info("[Halo-CampE-UEVR] NAVDUMP: sweeping for Navpoint/Waypoint/HudDataAsset "
                         "objects...");

    // Pass 1: the live objects, so ownership (which actor/asset holds what) reads from the log.
    // Pass 2 input: the unique classes seen, dumped once each.
    // 20, not 12: the first run capped out on exactly the two classes that mattered
    // (HaloUINavpointsManager and the widget class itself sit LATE in the object array).
    API::UStruct* classes[20] = {};
    int n_classes = 0;
    int logged = 0;
    const int32_t n = arr->get_object_count();
    for (int32_t i = 0; i < n && logged < 64; ++i) {
        auto* o = arr->get_object(i);
        if (o == nullptr) continue;
        const std::wstring cn = class_name_of(o);
        if (cn.empty()) continue;
        const bool hit = cn.find(L"Navpoint") != std::wstring::npos
                      || cn.find(L"NavPoint") != std::wstring::npos
                      || cn.find(L"Waypoint") != std::wstring::npos
                      || cn.find(L"HudDataAsset") != std::wstring::npos;
        if (!hit) continue;
        auto* cls = o->get_class();
        if (cls != nullptr && o == cls->get_class_default_object()) continue;   // instances only

        std::string path;
        for (API::UObject* p = o; p != nullptr; p = p->get_outer()) {
            const auto* fn = p->get_fname();
            path = (fn != nullptr ? narrow(fn->to_string()) : std::string("?")) +
                   (path.empty() ? "" : "." + path);
        }
        API::get()->log_info("[Halo-CampE-UEVR] NAVDUMP: %s  |  %s", narrow(cn).c_str(), path.c_str());
        ++logged;

        if (cls != nullptr && n_classes < 20) {
            bool seen = false;
            for (int k = 0; k < n_classes; ++k) { if (classes[k] == (API::UStruct*)cls) { seen = true; break; } }
            if (!seen) classes[n_classes++] = (API::UStruct*)cls;
        }
    }

    for (int k = 0; k < n_classes; ++k) {
        nav_dump_struct_chain(classes[k], 3);
    }
    API::get()->log_info("[Halo-CampE-UEVR] NAVDUMP: done (%d object(s), %d class(es))",
                         logged, n_classes);
}

#endif  // HALO_VR_DEV

}  // namespace halo
