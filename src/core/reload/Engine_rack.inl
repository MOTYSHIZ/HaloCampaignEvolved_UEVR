// core reload engine (runs while reloadvr or slidevr is on): the slide rack, the gesture frame helpers, the montage, slide fire, chamber and phantom, the slide copy and the slide parts. The gesture frame helpers and the reload press/co-op helpers here are shared with the manual reload.
// Textual fragment, included by core/reload/ReloadEngine.cpp in its anonymous namespace, after the magazine block. Moved verbatim; not compiled on its own.
// ---- THE SLIDE RACK (slidevr). The pistol's slide is node `slidenode` of the weapon object's
// own node block (BlamPalette publishes its world position and forward every sim tick, and
// applies g_slide_pull to it). Here, on the game thread: the off hand's grip closing within
// slideradius of the slide latches it; the pull is the hand's travel since the grab projected
// onto -forward (the barrel, muzzle-ward being +forward), clamped to slidetravel; releasing the
// grip springs it home. Distances are compared in UE centimetres: the hand goes room->world
// through the holster transform, the slide comes Blam->UE by the 304.8 unit with Y negated.
// The slide PART's rendered centre in UE world cm, published by the parts rebuild each tick
// (valid only while a slide part exists). The grab zone lives there (slide_zone=1).
std::atomic<bool>  g_sl_part_valid{false};
// PER-WEAPON ZONE PUSHBACK (from the headset, 2026-09-04): "@back=0.03" on a slidebones entry, metres
// further back along the barrel from the part's centre, added to the global slidezoneback. The
// magnum's serrations sit behind Slide_M's centre; nothing else needed the push.
std::atomic<float> g_sl_zone_back{0.0f};
std::atomic<float> g_sl_zone_up{0.0f};
std::atomic<float> g_sl_zone_right{0.0f};
std::atomic<bool>  g_sl_zone_reload_only{false};   // @reloadonly: the rack works only in the reload state (the shotgun's pump is its foregrip)
std::atomic<bool>  g_sl_zone_every_shot{false};    // @everyshot: every shot locks the gun until the rack (a pump between shots, from the headset 2026-09-06)
std::atomic<bool>  g_sl_zone_pump{false};          // @pump: back all the way then FORWARD all the way is the cycle; no grip release, no re-grab
std::atomic<bool>  g_sl_zone_pull_down{false};     // @pulldown: the pull is measured DOWNWARD, not rearward (the launcher's clamp is pulled down)
std::atomic<float> g_sl_insert{-1.0f};             // @insert=m: this weapon's own mag-in start below the seat (-1 = the global reloadinsert)
float reload_insert_for_weapon() { const float v = g_sl_insert.load(std::memory_order_relaxed); return v >= 0.0f ? v : g_cfg.reload_insert; }
std::atomic<float> g_sl_part_x{0.0f}, g_sl_part_y{0.0f}, g_sl_part_z{0.0f};
bool  s_sl_held = false;
bool  s_sl_racked = false;
bool  s_sl_grip_prev = false;
// THE MANUAL LOOP (slidelockreload): after a magazine seats, the slide is held locked back
// (the AnimBP's two-state ammunition pose, frame 0) and the trigger is dead until the player
// racks it -- pulled to the end and released. Then the pose goes back to what the game set
// (captured the moment the reload's ammo landed, so it is the weapon's own loaded frame, not a
// constant) and the gun fires. Mag in -> rack -> fire. Local presentation plus a local trigger
// block; the game's reload and ammo are untouched, so co-op is untouched.
bool    s_sl_th = false;          // the rack is driven by the two-hand SUPPORT hand: grabbed while holding, completed on the forward return
bool    s_sl_lock_pending = false;
int32_t s_sl_seen_frame = 0;   // last non-zero ammo frame seen on this weapon while idle
bool    s_sl_locked_back = false; // an EMPTY gun's slide stays back from the drop until the rack (the phantom flag no longer covers it)
// A LOADED CHAMBER (from the headset, 2026-09-05: "phantom round doesn't fire"): the sim refuses the
// trigger while its own reload runs, so a press at the drop would eat the chambered shot. On a
// gun with a round in it the press waits for that shot (150 ms after it, so the sim fires
// first) or for the seat, whichever comes first. An empty gun still presses at the drop.
bool      s_sl_press_pending = false;
long long s_sl_press_due_at = 0;
bool      s_sl_pressed_early = false;   // reload_press_at 1: the press went out at the drop
long long s_sl_press_at = 0;            // when the plugin last pressed the game's reload
int32_t s_sl_lock_frame = 0;
// The loaded (forward) ammo frame: the one captured during the lock, else the last one seen idle,
// else 1 -- the pose is two-state and anything but 0 renders forward.
int32_t slide_forward_frame() { if (s_sl_lock_frame != 0) return s_sl_lock_frame; if (s_sl_seen_frame != 0) return s_sl_seen_frame; return 1; }
bool    s_sl_rack_done = false;
bool    s_sl_empty_at_drop = false;   // the pistol was empty (slide locked back) when the mag left
bool    s_sl_reload_due = false;      // SLIDECHAMBER: a seated mag waiting for the rack to fire the reload
bool    s_true_empty = false;         // SLIDEPHANTOM: the gun is empty while the sim reads one round
// ONE CHAMBERED ROUND. With the magazine out a pistol fires exactly once (the round already in
// the chamber), then nothing until a fresh magazine is seated. Counted on trigger press edges
// while the reload is in flight; the press itself is seen through the ForceTube's fire stamp.
int     s_sl_chamber_left = 0;
bool    s_sl_trig_prev = false;
// The weapon AnimBP's ammunition frame: 0 = empty / slide locked back, else loaded. -1 = none.
int32_t slide_ammo_frame() {
    auto* animbp = reload_weapon_anim_instance();
    if (animbp == nullptr) return -1;
    auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame");
    if (p == nullptr || IsBadReadPtr(p, sizeof(int32_t))) return -1;
    return *p;
}
Vec3  s_sl_grab_ue{};
long long s_sl_release_at = 0;
float s_sl_release_pull = 0.0f;
inline Vec3 blam_to_ue_cm(float bx, float by, float bz) { return Vec3{bx * 304.8f, -by * 304.8f, bz * 304.8f}; }
inline Vec3 blam_dir_to_ue(float bx, float by, float bz) { return Vec3{bx, -by, bz}; }
std::string trim_cfg(const char* v);   // defined with the montage tools below
// ---- THE GESTURE FRAME (reloadframe, Config.hpp). The world-space tests compare weapon
// component positions read on the GAME tick against a hand mapped through the RENDERED camera;
// while sprinting those time bases are up to a tick apart and the measured distance oscillates
// by the camera's per-tick travel. These helpers give the tests one time base.
namespace {
Vec3 s_gc_cam{}; bool s_gc_have = false; unsigned long long s_gc_at = ~0ull;
Vec3 s_gc_prev{}; bool s_gc_prev_have = false; float s_gc_travel = 0.0f;
}
// The GAME camera, fetched at most once per ~tick (ms-keyed; engine ticks are ~31 ms apart) on
// the game thread -- the same time base as every K2_GetComponentLocation/Bounds the tests read.
bool gesture_game_cam(Vec3* out) {
    const unsigned long long nowms = GetTickCount64();
    if (nowms != s_gc_at) {
        s_gc_at = nowms;
        s_gc_have = false;
        if (auto* pc = API::get()->get_player_controller(0)) {
            if (auto** cm = pc->get_property_data<API::UObject*>(L"PlayerCameraManager")) {
                if (!IsBadReadPtr(cm, sizeof(void*)) && *cm != nullptr) {
                    Vec3 v{};
                    if (call_ret_vec3(*cm, L"GetCameraLocation", &v)) { s_gc_cam = v; s_gc_have = true; }
                }
            }
        }
        if (s_gc_have) {
            if (s_gc_prev_have) {
                const float dx = s_gc_cam.x - s_gc_prev.x, dy = s_gc_cam.y - s_gc_prev.y, dz = s_gc_cam.z - s_gc_prev.z;
                float t = std::sqrt(dx * dx + dy * dy + dz * dz) * 0.01f;   // cm -> m this tick
                if (t > 0.30f) t = 0.30f;   // a teleport or level load is not running
                s_gc_travel = t;
            }
            s_gc_prev = s_gc_cam; s_gc_prev_have = true;
        } else { s_gc_prev_have = false; s_gc_travel = 0.0f; }
    }
    if (s_gc_have && out != nullptr) *out = s_gc_cam;
    return s_gc_have;
}
// reloadframe=2: the pad added to the seat/rack gates, metres -- the camera's per-tick travel,
// which is the worst case of the phase error the legacy transform carries. 0 in every other mode.
float reload_gate_pad_m() {
    if (g_cfg.reload_frame != 2) return 0.0f;
    gesture_game_cam(nullptr);
    return s_gc_travel;
}
// The hand (room) into world for the seat tests: the tick's own camera when reloadframe=1 and it
// resolves, the legacy rendered-frame transform otherwise.
// With the snapshot in force the camera is the SNAPSHOT'S, never a fresh fetch: gesture_game_cam is
// ms-keyed, which makes it one tick old at worst rather than one instant, and a transform is the
// one place where "the current camera" used to enter a decision it had no business entering.
Vec3 reload_hand_world(const Vec3& hand_room, const Vec3& head_room) {
    if (zone_snapshot_on() && s_snap.cam_ok) return holster_room_to_world_at(hand_room, head_room, s_snap.cam);
    Vec3 cam{};
    if (g_cfg.reload_frame == 1 && gesture_game_cam(&cam)) return holster_room_to_world_at(hand_room, head_room, cam);
    return holster_room_to_world(hand_room, head_room);
}
// The inverse, same frame choice -- the exact mirror of reload_hand_world, so a value pushed
// through both comes back bit-stable.
Vec3 reload_world_room(const Vec3& world, const Vec3& head_room) {
    if (zone_snapshot_on() && s_snap.cam_ok) return holster_world_to_room_at(world, head_room, s_snap.cam);
    Vec3 cam{};
    if (g_cfg.reload_frame == 1 && gesture_game_cam(&cam)) return holster_world_to_room_at(world, head_room, cam);
    return holster_world_to_room(world, head_room);
}
// THE WELL RIDES THE HAND (zonehandrel, Config.hpp). The magazine component's position is
// re-expressed in the aim hand's frame, low-passed, and rebuilt from the live hand pose, so the
// seat point tracks the RENDERED gun instead of the game's own sprint animation. The alpha
// (0.2/tick at ~32 Hz, ~150 ms) kills the bob but follows a real weapon swap instantly through
// the snap guard.
// THE LOW PASS IS ZONESNAPSHOT'S BUSINESS NOW. 0.2/tick at ~32 Hz is a ~150 ms lag, and it was
// there to hide the frame mix; with one snapshot it can only put the lag back, so mode 1 opens it
// to 1.0. Mode 3 replaces it outright with a learn-then-hold, which is what actually answers the
// sprint animation (see zone_offset_hold below). Mode 0 keeps the inherited filter.
float zone_offset_alpha() { return (g_cfg.zone_snapshot == 0) ? 0.2f : 1.0f; }
// MODE 3: LEARN THE HAND-FRAME OFFSET, THEN HOLD IT. Learning is allowed only while the player is
// not moving the camera much (the sprint animation only plays while he is) and the offset is
// steady tick to tick. Both gates are MEASURED, not inferred: the camera's own per-tick travel is
// what gesture_game_cam already integrates from GetCameraLocation, and the steadiness is this
// value's own first difference. Four consecutive steady ticks before it is taken as learned, so a
// single quiet frame mid-sprint cannot re-learn a displaced offset.
struct ZoneHold { void* key = nullptr; Vec3 loc{}; bool have = false; int steady = 0; };
bool zone_offset_hold(ZoneHold& h, void* comp_key, const Vec3& loc, Vec3* out) {
    if (g_cfg.zone_snapshot != 3) return false;
    const float dx = loc.x - h.loc.x, dy = loc.y - h.loc.y, dz = loc.z - h.loc.z;
    const float step = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (comp_key != h.key || !h.have) { h.key = comp_key; h.loc = loc; h.have = true; h.steady = 0; *out = loc; return true; }
    const bool still  = s_snap.cam_travel <= g_cfg.zone_snapshot_still;
    const bool steady = step <= g_cfg.zone_snapshot_steady;
    if (still && steady) { if (h.steady < 4) ++h.steady; }
    else                 h.steady = 0;
    // Learn while the gates hold OR while nothing has been learned yet (the first seconds on a
    // fresh weapon must converge even if the player picked it up on the run); hold otherwise.
    if ((still && steady) || h.steady < 4) h.loc = loc;
    *out = h.loc;
    return true;
}
Vec3 reload_well_stabilize(void* comp_key, const Vec3& well_world, const Vec3& head_room) {
    if (g_cfg.zone_hand_rel == 0) return well_world;
    Vec3 ap{}; Vec3 f{}, u{}, r{};
    if (zone_snapshot_on()) {
        // THE SNAPSHOT'S AIM HAND, not a fresh read. This function used to call get_pose itself --
        // a SECOND pose read inside a decision whose other side (the component's world position)
        // had already been sampled, and a third read after the caller's own two.
        if (!s_snap.poses_ok) return well_world;
        ap = s_snap.aim; f = s_snap.f; u = s_snap.u; r = s_snap.r;
    } else {
        const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                              : API::VR::get_right_controller_index();
        Quat aq{};
        if (ridx < 0 || !get_pose(ridx, &ap, &aq, /*use_aim=*/true)) return well_world;
        const Quat af = placement_aim_fix(aq);
        f = quat_forward(af);
        u = quat_rotate(af, Vec3{0.0f, 1.0f, 0.0f});
        r = quat_rotate(af, Vec3{1.0f, 0.0f, 0.0f});
    }
    const Vec3 wr = reload_world_room(well_world, head_room);
    const Vec3 rel{wr.x - ap.x, wr.y - ap.y, wr.z - ap.z};
    const Vec3 loc{rel.x * r.x + rel.y * r.y + rel.z * r.z,
                   rel.x * u.x + rel.y * u.y + rel.z * u.z,
                   rel.x * f.x + rel.y * f.y + rel.z * f.z};
    static ZoneHold s_hold;
    static void* s_key = nullptr; static Vec3 s_loc{}; static bool s_have = false;
    if (!zone_offset_hold(s_hold, comp_key, loc, &s_loc)) {
        const float dx = loc.x - s_loc.x, dy = loc.y - s_loc.y, dz = loc.z - s_loc.z;
        if (comp_key != s_key || !s_have || dx * dx + dy * dy + dz * dz > 0.25f) { s_key = comp_key; s_loc = loc; s_have = true; }
        else { const float a = zone_offset_alpha(); s_loc.x += dx * a; s_loc.y += dy * a; s_loc.z += dz * a; }
    }
    const Vec3 room{ap.x + r.x * s_loc.x + u.x * s_loc.y + f.x * s_loc.z,
                    ap.y + r.y * s_loc.x + u.y * s_loc.y + f.y * s_loc.z,
                    ap.z + r.z * s_loc.x + u.z * s_loc.y + f.z * s_loc.z};
    return reload_hand_world(room, head_room);
}
// The held weapon's mesh stem, lowercased (SK_FuelRodCannon_Default -> fuelrodcannon), cached per
// weapon key and actor. The class name and the mesh name disagree on one weapon: BP_FP_FlakCannon
// renders the fuel rod cannon, so "FuelRod" in a list never matched its key.
std::string weapon_stem_lc_for(const std::string& key) {
    static std::string s_key; static API::UObject* s_actor = nullptr; static std::string s_stem; static long long s_at = 0;
    auto* actor = fp_weapon_actor();
    const long long nowt = now_ticks();
    if (key != s_key || actor != s_actor || (s_stem.empty() && nowt - s_at > ms_to_ticks(500))) {
        s_key = key; s_actor = actor; s_at = nowt;
        s_stem = narrow(ak_weapon_stem());
    }
    return s_stem;
}
// A weapon key (or its mesh stem) against a comma-separated list of substrings (case-insensitive, trimmed).
bool weapon_in_list(const char* list) {
    const std::string key = weapon_key();
    if (key.empty()) return false;
    std::string lk = key; for (auto& ch : lk) ch = (char)tolower((unsigned char)ch);
    const std::string stem = weapon_stem_lc_for(key);
    std::string tbl = trim_cfg(list);
    size_t pos = 0;
    while (pos <= tbl.size()) {
        size_t comma = tbl.find(',', pos); if (comma == std::string::npos) comma = tbl.size();
        std::string ent = tbl.substr(pos, comma - pos);
        while (!ent.empty() && (unsigned char)ent.back() <= ' ') ent.pop_back();
        while (!ent.empty() && (unsigned char)ent.front() <= ' ') ent.erase(ent.begin());
        for (auto& ch : ent) ch = (char)tolower((unsigned char)ch);
        if (!ent.empty() && (lk.find(ent) != std::string::npos || (!stem.empty() && stem.find(ent) != std::string::npos))) return true;
        if (comma >= tbl.size()) break;
        pos = comma + 1;
    }
    return false;
}
bool slide_weapon_ok() { return weapon_in_list(g_cfg.slide_weapons); }
// The rack part the native mode found for the held weapon (set by the parts code below). A
// weapon whose rack part is missing must never be locked back or phantomed: nothing could
// finish its reload.
bool g_slide_rack_found = false;
bool slide_rack_available() { return g_slide_rack_found; }
// The pistol rules (lock-back, phantom round, rack-fired reload) apply only to these.
bool slide_chamber_ok() { return weapon_in_list(g_cfg.slide_chamber_weapons); }
void slide_haptic(float dur, float amp) {
    const bool off_right = g_cfg.aim_left_hand;
    API::VR::trigger_haptic_vibration(0.0f, dur, 0.0f, amp,
                                      off_right ? API::VR::get_right_joystick_source()
                                                : API::VR::get_left_joystick_source());
}
// THE GAME-THREAD WRITE. The sim rebuilds the slide node every tick (readback measured 2026-09-03),
// and the mesh sync reads it on the game thread during component ticks -- AFTER this pre-engine
// tick. So the pull (and the dev poke) are applied HERE, where the write is guaranteed to sit
// between the rebuild and the read. The sim-side write stays as a second bite at the same tick.
void slide_node_write_tick() {
    if (blam_capture_hook_active()) return;   // the capture pre-hook owns the write (renders)
    const uintptr_t addr = g_slide_node_addr.load(std::memory_order_relaxed);
    if (addr == 0 || IsBadWritePtr((void*)addr, 52)) return;
    struct NodeM { float scale; float fx, fy, fz; float lx, ly, lz; float ux, uy, uz; float px, py, pz; };
    auto* n = reinterpret_cast<NodeM*>(addr);
    float along = 0.0f;
    const float pull = g_slide_pull.load(std::memory_order_relaxed);
    if (pull > 0.0f && std::isfinite(pull)) along -= pull;
    if (g_cfg.wpn_node_poke >= 0) along += g_cfg.wpn_node_poke_amt;
    if (along == 0.0f) return;
    n->px += n->fx * along; n->py += n->fy * along; n->pz += n->fz * along;
    if (g_cfg.slide_log) {
        static long long s_said = 0;
        if (now_ticks() - s_said > ms_to_ticks(1000)) {
            s_said = now_ticks();
            API::get()->log_info("[Halo-CampE-UEVR] SLIDE game-thread write: along=%.4f pos=(%.3f %.3f %.3f)", along, n->px, n->py, n->pz);
        }
    }
}

void slide_update(const Vec3& head) {
    auto release = [&](const char* why) {
        if (s_sl_held) {
            s_sl_release_pull = g_slide_pull.load(std::memory_order_relaxed);
            s_sl_release_at = now_ticks();
            if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE released (%s) at pull %.4f", why, s_sl_release_pull);
            if (s_sl_racked) { slide_haptic(0.06f, 0.9f); s_sl_rack_done = true; ak_step_sound("rack"); }
        }
        s_sl_held = false; s_sl_racked = false; s_sl_th = false;
    };
    // The chambered round's shot: a trigger press edge while the reload is in flight spends it.
    {
        const long long since = now_ticks() - g_ft_fire_at.load(std::memory_order_relaxed);
        const bool trig = since >= 0 && since < ms_to_ticks(40);
        if (trig && !s_sl_trig_prev && s_reload != ReloadState::Idle && s_sl_chamber_left > 0) {
            --s_sl_chamber_left;
            if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE chambered round fired, trigger now dead until the seat");
            if (s_sl_press_pending) { s_sl_press_pending = false; s_sl_press_due_at = now_ticks() + ms_to_ticks(150); }
            if (s_sl_chamber_left <= 0) {
                // The gun is empty NOW: the slide locks back and stays there until the rack after
                // the seat (from the headset, 2026-09-06: the chambered shot did not lock the slide).
                s_sl_empty_at_drop = true; s_sl_locked_back = true;
                if (g_cfg.slide_log || g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE chamber empty after the shot: locked back, the seat will want a rack");
            }
        }
        s_sl_trig_prev = trig;
    }
    // The lock-back after a reload: hold the ammunition pose at frame 0 until a rack completes.
    // The game's own loaded frame is captured the first time it reads non-zero after the seat
    // (the reload's ammo landing), so the restore is the weapon's value, not a guess.
    // THE LOADED POSE (2026-09-05): PrimaryAmmunition_ExplicitFrame is two-state -- 0 = slide
    // locked back, any other value = forward (measured 1/2/8/28/56/59; it is an animation frame,
    // NOT the round count). With the press at the drop the game's own loaded frame may never be
    // seen (the state hold keeps its reload from landing one), so the last non-zero frame seen on
    // this weapon while idle is remembered, and 1 is the last resort: anything but 0 is forward.
    {
        static std::string s_seen_key; static int32_t s_seen_frame = 0;
        const std::string wk = weapon_key();
        if (wk != s_seen_key) { s_seen_key = wk; s_seen_frame = 0; }
        if (s_reload == ReloadState::Idle && !s_sl_lock_pending && !s_sl_pressed_early) {
            if (auto* animbp = reload_weapon_anim_instance())
                if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame"))
                    if (!IsBadReadPtr(p, sizeof(int32_t)) && *p != 0) s_seen_frame = *p;
        }
        s_sl_seen_frame = s_seen_frame;
    }
    {
        static bool s_ep_held = false;
        if (s_sl_pressed_early && s_reload != ReloadState::Idle && !s_sl_lock_pending) {
            if (auto* animbp = reload_weapon_anim_instance()) {
                if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame")) {
                    // An empty gun shows the slide back (0); a gun with a round chambered keeps the
                    // pose it had before the drop (the frame is the slide, not the counter).
                    if (!IsBadWritePtr(p, sizeof(int32_t))) { if (*p != 0 && s_sl_lock_frame == 0) s_sl_lock_frame = *p; *p = s_sl_empty_at_drop ? 0 : slide_forward_frame(); s_ep_held = true; }
                }
            }
        } else if (s_ep_held) {
            s_ep_held = false;
            if (!s_sl_lock_pending) {   // a chambered-round reload: the seat alone ends it, the pose goes forward now
                if (auto* animbp = reload_weapon_anim_instance())
                    if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame"))
                        if (!IsBadWritePtr(p, sizeof(int32_t))) *p = slide_forward_frame();
            }
        }
    }
    // Only the rack ends this lock, and with slidevr off nothing can rack (the gate below releases
    // the hand): release it as racked, so the pose goes forward and a seated magazine's reload goes
    // out instead of s_sl_reload_due waiting forever behind a frozen slide.
    if (!reload_rack_available() && s_sl_lock_pending) {
        s_sl_rack_done = true;
        if (reload_weapon_anim_instance() == nullptr) { s_sl_lock_pending = false; s_sl_rack_done = false; s_sl_locked_back = false; }
    }
    if (s_sl_lock_pending) {
        if (auto* animbp = reload_weapon_anim_instance()) {
            if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame")) {
                if (!IsBadWritePtr(p, sizeof(int32_t))) {
                    if (s_sl_rack_done) {
                        *p = slide_forward_frame();
                        s_sl_lock_pending = false; s_sl_rack_done = false; s_sl_locked_back = false;
                        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE rack complete, pose restored to frame %d, fire unblocked", s_sl_lock_frame);
                    } else {
                        if (*p != 0 && s_sl_lock_frame == 0) s_sl_lock_frame = *p;
                        *p = 0;
                    }
                }
            }
        }
    } else {
        s_sl_rack_done = false;
    }
    // The spring home after a release: a short ease from the released pull to zero.
    if (!s_sl_held && s_sl_release_at != 0) {
        const float ms = (float)std::chrono::duration_cast<std::chrono::milliseconds>(
            clock_t_::duration(now_ticks() - s_sl_release_at)).count();
        const float t = std::fmin(1.0f, ms / 90.0f);
        g_slide_pull.store(s_sl_release_pull * (1.0f - t), std::memory_order_relaxed);
        if (t >= 1.0f) { s_sl_release_at = 0; g_slide_pull.store(0.0f, std::memory_order_relaxed); }
    }
    if (!reload_rack_available() || !g_slide_node_valid.load(std::memory_order_relaxed) || !slide_weapon_ok()) {
        release("unavailable"); return;
    }
    // THE ZONE IS AIM-HAND-RELATIVE, NOT THE NODE'S WORLD POSITION. The published node sits on
    // the third-person gun in the biped's hand (measured: never nearer than ~40 cm to the hand
    // actually on the rendered slide); the mesh sync copies the node's MOTION into the
    // first-person mesh, not its place. So the slide is where the rendered gun is: slideoff
    // metres from the aim hand in the aim-fixed frame (right, up, forward), the same recipe as
    // the reload well's fallback. The pull is measured along the aim direction in room space.
    // ONE SNAPSHOT (zonesnapshot, doctrine and the measured numbers in ConfigFields.inl). With the
    // key on, every quantity below comes from the tick's own snapshot -- the head, both hands, the
    // camera and the rack part's world centre, all sampled at one instant and carrying one
    // sequence number -- and nothing in this path reads a newest value. With the key off it is
    // the inherited pair of fresh pose reads against a part position published one tick later.
    const bool snap = zone_snapshot_on();
    Vec3 hp{}; Quat hq{};
    Vec3 ap{}; Quat aq{};
    Vec3 f{}, u{}, r{};
    Vec3 hd = head;
    if (snap) {
        if (!s_snap.poses_ok) { release("no hand"); return; }
        hp = s_snap.off; hq = s_snap.off_q;
        ap = s_snap.aim; aq = s_snap.aim_q;
        f = s_snap.f; u = s_snap.u; r = s_snap.r;
        hd = s_snap.head;
        // MODE 2: THE ZONE COMES OFF THE DRAWN GUN. The placement owns the rendered pose, so the
        // frame the slide the player can SEE sits in is the drawn weapon's, which the snapshot
        // composed the way aimbore composes the drawn barrel. slideoff is then read in the GUN's
        // own axes rather than the hand's: quat_forward is the barrel (UE +X under the room map
        // x=ue.y, y=ue.z, z=-ue.x), room +Y is the gun's up (UE +Z) and room +X its right (UE +Y).
        // The game component leaves the path entirely -- the slide_zone==1 branch below is skipped.
        if (g_cfg.zone_snapshot == 2 && s_snap.wpn_ok) {
            const Quat wq = s_snap.wpn_q;
            f = quat_forward(wq);
            u = quat_rotate(wq, Vec3{0.0f, 1.0f, 0.0f});
            r = quat_rotate(wq, Vec3{1.0f, 0.0f, 0.0f});
        }
    } else {
        const auto idx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                             : API::VR::get_left_controller_index();
        const auto ridx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                              : API::VR::get_right_controller_index();
        if (idx < 0 || ridx < 0 || !get_pose(idx, &hp, &hq, /*use_aim=*/false) ||
            !get_pose(ridx, &ap, &aq, /*use_aim=*/true)) { release("no hand"); return; }
        const Quat af = placement_aim_fix(aq);
        f = quat_forward(af);
        u = quat_rotate(af, Vec3{0.0f, 1.0f, 0.0f});
        r = quat_rotate(af, Vec3{1.0f, 0.0f, 0.0f});
    }
    Vec3 zone{ap.x + r.x * g_cfg.slide_off[0] + u.x * g_cfg.slide_off[1] + f.x * g_cfg.slide_off[2],
              ap.y + r.y * g_cfg.slide_off[0] + u.y * g_cfg.slide_off[1] + f.y * g_cfg.slide_off[2],
              ap.z + r.z * g_cfg.slide_off[0] + u.z * g_cfg.slide_off[1] + f.z * g_cfg.slide_off[2]};
    const bool drawn_gun_zone = snap && g_cfg.zone_snapshot == 2 && s_snap.wpn_ok;
    const bool part_ok = snap ? s_snap.part_ok : g_sl_part_valid.load(std::memory_order_relaxed);
    if (g_cfg.slide_zone == 1 && part_ok && !drawn_gun_zone) {
        // The part's own rendered centre, brought into room space, then a little further back
        // along the aim direction to where the serrations are.
        const Vec3 pw = snap ? s_snap.part
                             : Vec3{g_sl_part_x.load(std::memory_order_relaxed), g_sl_part_y.load(std::memory_order_relaxed), g_sl_part_z.load(std::memory_order_relaxed)};
        Vec3 pr = reload_world_room(pw, hd);
        // HAND-ANCHORED (zonehandrel, Config.hpp): the part offset lives in the aim hand's
        // frame, low-passed; the zone rebuilds from the live hand, so it rides the gun you SEE
        // and not the game's sprint animation. Snap guard for weapon swaps.
        if (g_cfg.zone_hand_rel != 0) {
            const Vec3 rel{pr.x - ap.x, pr.y - ap.y, pr.z - ap.z};
            const Vec3 loc{rel.x * r.x + rel.y * r.y + rel.z * r.z,
                           rel.x * u.x + rel.y * u.y + rel.z * u.z,
                           rel.x * f.x + rel.y * f.y + rel.z * f.z};
            static ZoneHold s_zhold;
            static Vec3 s_zl{}; static bool s_zh = false;
            // MODE 3 learns this offset while the sprint animation is not playing and holds it;
            // every other mode keeps the filter, opened to 1.0 by zone_offset_alpha under the
            // snapshot because the lag it bought was only hiding the frame mix.
            if (!zone_offset_hold(s_zhold, s_snap.part_key, loc, &s_zl)) {
                const float dx = loc.x - s_zl.x, dy = loc.y - s_zl.y, dz = loc.z - s_zl.z;
                if (!s_zh || dx * dx + dy * dy + dz * dz > 0.25f) { s_zl = loc; s_zh = true; }
                else { const float a = zone_offset_alpha(); s_zl.x += dx * a; s_zl.y += dy * a; s_zl.z += dz * a; }
            }
            pr = Vec3{ap.x + r.x * s_zl.x + u.x * s_zl.y + f.x * s_zl.z,
                      ap.y + r.y * s_zl.x + u.y * s_zl.y + f.y * s_zl.z,
                      ap.z + r.z * s_zl.x + u.z * s_zl.y + f.z * s_zl.z};
        }
        const float back = g_cfg.slide_zone_back + g_sl_zone_back.load(std::memory_order_relaxed);
        const float up = g_sl_zone_up.load(std::memory_order_relaxed), right = g_sl_zone_right.load(std::memory_order_relaxed);
        zone = Vec3{pr.x - f.x * back + u.x * up + r.x * right, pr.y - f.y * back + u.y * up + r.y * right, pr.z - f.z * back + u.z * up + r.z * right};
    }
    // The zone dot (slidemarker): where the grab is, so slideoff can be set by eye.
    {
        static TrackedObject s_dot;
        static long long s_dot_try = 0;
        auto* d = s_dot.get();
        if (g_cfg.slide_marker) {
            if (d == nullptr && now_ticks() - s_dot_try > ms_to_ticks(2000)) {
                s_dot_try = now_ticks();
                if (auto* owner = API::get()->get_local_pawn(0)) {
                    static const wchar_t* kDot[] = { L"StaticMesh /Engine/BasicShapes/Sphere.Sphere" };
                    { const double ds = (double)g_cfg.slide_marker_size; d = marker_spawn_list(owner, kDot, 1, ds, ds, ds); }
                    if (d != nullptr) { marker_tint(d, g_cfg.slide_marker_color); s_dot.set(d); }
                    API::get()->log_info("[Halo-CampE-UEVR] SLIDE zone dot %s", d ? "spawned" : "FAILED to spawn (no sphere mesh?)");
                }
            }
            // THE DOT IS THE ZONE, from the same snapshot the grab test uses: one room point,
            // registered for the render pass, and its tick placement pushed through the same
            // transform the test's own distance was measured in (reload_hand_world, the snapshot's
            // camera) rather than the rendered-frame one.
            if (d != nullptr) { holster_marker_place(d, reload_hand_world(zone, hd)); holster_marker_show(d, true); marker_render_anchor(d, zone); }
        } else if (d != nullptr) {
            holster_marker_show(d, false);
            marker_render_drop(d);
        }
    }
    const bool grip = holster_grip_held(g_cfg.aim_left_hand);
    const bool pressed = grip && !s_sl_grip_prev;
    s_sl_grip_prev = grip;
    const float dx = hp.x - zone.x, dy = hp.y - zone.y, dz = hp.z - zone.z;
    const float dist_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    // The rack is LIVE when the weapon has a part and, for a @reloadonly weapon, only in the
    // reload state (a shell in, the mag out, or the lock waiting for the rack).
    const bool rack_live = (g_slide_rack_found || g_sl_part_valid.load(std::memory_order_relaxed))
                        && (!g_sl_zone_reload_only.load(std::memory_order_relaxed) || s_sl_lock_pending || s_reload != ReloadState::Idle || s_true_empty || s_sl_locked_back);
    g_slide_zone_hot.store(rack_live && dist_m <= g_cfg.slide_radius + reload_gate_pad_m(), std::memory_order_relaxed);
    if (g_cfg.slide_log) {
        static uint32_t s_n = 0;
        if ((s_n++ % 30u) == 0u)
            API::get()->log_info("[Halo-CampE-UEVR] SLIDE hand-to-zone=%.0fcm held=%d pull=%.4f | snap=%u mode=%d camtravel=%.0fcm src=%s",
                                 dist_m * 100.0f, (int)s_sl_held, g_slide_pull.load(),
                                 snap ? s_snap.seq : 0u, g_cfg.zone_snapshot, s_snap.cam_travel * 100.0f,
                                 drawn_gun_zone ? "drawn gun" : (part_ok && g_cfg.slide_zone == 1 ? "part" : "slideoff"));
    }
    // THE PUMP WITH THE HOLD (from the headset, 2026-09-06: two-handing and pumping work together): while
    // the two-hand hold is latched and the rack is live, the support hand entering the zone
    // takes the rack without a new press; pulling back racks it, and the FORWARD return completes
    // it (there is no grip release to wait for). A hand still in the zone grabs again for the
    // next cycle.
    if (!s_sl_held && grip && rack_live && (two_hand_latched() || g_sl_zone_pump.load(std::memory_order_relaxed)) && dist_m <= g_cfg.slide_radius * 1.5f && s_reload == ReloadState::Idle && !holster_offhand_busy()) {
        s_sl_held = true; s_sl_th = true; s_sl_racked = false; s_sl_grab_ue = hp; s_sl_release_at = 0;
        slide_haptic(0.04f, 0.5f);
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE grabbed by the support hand (two-hand hold) at %.0fcm", dist_m * 100.0f);
    }
    if (!s_sl_held) {
        if (pressed && dist_m <= g_cfg.slide_radius && rack_live) {
            const bool th   = two_hand_latched();
            const bool busy = holster_offhand_busy();
            if (s_reload == ReloadState::Idle && !th && !busy) {
                s_sl_held = true; s_sl_racked = false; s_sl_grab_ue = hp; s_sl_release_at = 0;
                slide_haptic(0.04f, 0.5f);
                if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE grabbed at %.0fcm", dist_m * 100.0f);
            } else if (g_cfg.slide_log) {
                // A grip inside the zone that did not take: name the gate, do not guess it.
                API::get()->log_info("[Halo-CampE-UEVR] SLIDE grab REFUSED at %.0fcm: reload=%s twohand=%d offhand-busy=%d",
                                     dist_m * 100.0f, state_name(s_reload), (int)th, (int)busy);
            }
        }
        return;
    }
    if (!grip) { release("grip opened"); return; }
    // Pull: hand travel since the grab along -aim forward (rearward), metres -> Blam units.
    const float tx = hp.x - s_sl_grab_ue.x, ty = hp.y - s_sl_grab_ue.y, tz = hp.z - s_sl_grab_ue.z;
    float pull_cm = g_sl_zone_pull_down.load(std::memory_order_relaxed)
                  ? -(tx * u.x + ty * u.y + tz * u.z) * 100.0f * g_cfg.slide_sign    // @pulldown: downward travel
                  : -(tx * f.x + ty * f.y + tz * f.z) * 100.0f * g_cfg.slide_sign;
    if (pull_cm < 0.0f) pull_cm = 0.0f;
    const float travel_cm = g_cfg.slide_travel * 304.8f;
    if (pull_cm > travel_cm) pull_cm = travel_cm;
    g_slide_pull.store(pull_cm / 304.8f, std::memory_order_relaxed);
    // A locked-back slide only needs the slide RELEASE: a quarter of the travel and let go.
    const bool at_end = pull_cm >= travel_cm * (s_sl_lock_pending ? 0.25f : 0.92f);
    if (at_end && !s_sl_racked) {
        s_sl_racked = true;
        slide_haptic(0.08f, 1.0f);
        ak_step_sound("rackback");
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE racked (%.1fcm)", pull_cm);
    } else if (!at_end && s_sl_racked && pull_cm < travel_cm * (s_sl_th ? 0.15f : 0.5f)) {
        if (s_sl_th) { release("pump returned"); return; }   // back all the way, then FORWARD all the way: the cycle
        s_sl_racked = false;
    }
}

// ---- SLIDEMONTAGE. The first-person slide lives in the weapon's Animation Blueprint, so the
// rack poses it there: PlaySlotAnimationAsDynamicMontage on the weapon's anim instance with a
// real weapon sequence at play rate ~0, then Montage_SetPosition every tick to the time the hand
// asks for. Parameters are placed by NAME through the UFunction's own property offsets (logged
// once), never by an assumed layout. A montage only poses bones through a Slot node the ABP
// actually has; the ABP's slot-named node properties are listed once so the slot name is a
// fact, not a guess.
std::string trim_cfg(const char* v) {
    std::string s = v;
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
    return s;
}
TrackedObject s_sm_seq, s_sm_mont, s_sm_inst;
std::string   s_sm_seq_last;
float         s_sm_len = 0.0f;
bool          s_sm_active = false;
bool          s_sm_slots_listed = false;
bool sm_put(API::UFunction* fn, uint8_t* p, size_t cap, const wchar_t* name, const void* v, size_t n, bool* ok) {
    auto* pr = fn->find_property(name);
    if (pr == nullptr) { *ok = false; API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: no parameter '%ls'", name); return false; }
    const int32_t off = pr->get_offset();
    if (off < 0 || (size_t)off + n > cap) { *ok = false; return false; }
    memcpy(p + off, v, n);
    return true;
}
void sm_list_slots(API::UObject* animbp) {
    if (s_sm_slots_listed) return;
    s_sm_slots_listed = true;
    int n = 0;
    for (API::UStruct* st = animbp->get_class(); st != nullptr && n < 40; st = st->get_super_struct()) {
        if (IsBadReadPtr(st, sizeof(void*))) break;
        for (API::FField* f = st->get_child_properties(); f != nullptr && n < 40; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            const auto* fn = f->get_fname();
            auto* fc = f->get_class();
            if (fn == nullptr || fc == nullptr) continue;
            const std::wstring nm = fn->to_string();
            if (nm.find(L"Slot") == std::wstring::npos) continue;
            const int32_t off = static_cast<API::FProperty*>(f)->get_offset();
            // FAnimNode_Slot begins with an FPoseLink; the SlotName FName follows it. Two
            // candidate offsets are printed and the one that reads as a name is the answer.
            std::wstring a = L"?", b = L"?";
            const uint8_t* base = reinterpret_cast<const uint8_t*>(animbp) + off;
            if (!IsBadReadPtr(base, 0x28)) {
                a = reinterpret_cast<const API::FName*>(base + 0x10)->to_string();
                b = reinterpret_cast<const API::FName*>(base + 0x18)->to_string();
            }
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE slot-node %ls (%ls@0x%X): name@+0x10='%ls' name@+0x18='%ls'",
                                 nm.c_str(), fc->get_name().c_str(), (unsigned)off, a.c_str(), b.c_str());
            ++n;
        }
    }
    if (n == 0) API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: no property with 'Slot' in its name on %ls", class_name_of(animbp).c_str());
}
API::UObject* sm_play(API::UObject* animbp, API::UObject* seq, float t) {
    auto* fn = animbp->get_class()->find_function(L"PlaySlotAnimationAsDynamicMontage");
    if (fn == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: PlaySlotAnimationAsDynamicMontage not found on %ls", class_name_of(animbp).c_str()); return nullptr; }
    alignas(16) uint8_t p[0x100] = {0};
    bool ok = true;
    const std::string slot = trim_cfg(g_cfg.slide_slot);
    const std::wstring wslot(slot.begin(), slot.end());
    API::FName slotname = make_fname(wslot.c_str());
    const float bin = 0.0f, bout = 0.05f, rate = 0.001f, trig = -1.0f;
    const int32_t loops = 1;
    sm_put(fn, p, sizeof(p), L"Asset", &seq, sizeof(void*), &ok);
    sm_put(fn, p, sizeof(p), L"SlotNodeName", &slotname, sizeof(int32_t) * 2, &ok);
    sm_put(fn, p, sizeof(p), L"BlendInTime", &bin, sizeof(float), &ok);
    sm_put(fn, p, sizeof(p), L"BlendOutTime", &bout, sizeof(float), &ok);
    sm_put(fn, p, sizeof(p), L"InPlayRate", &rate, sizeof(float), &ok);
    sm_put(fn, p, sizeof(p), L"LoopCount", &loops, sizeof(int32_t), &ok);
    sm_put(fn, p, sizeof(p), L"BlendOutTriggerTime", &trig, sizeof(float), &ok);
    sm_put(fn, p, sizeof(p), L"InTimeToStartMontageAt", &t, sizeof(float), &ok);
    auto* ret = fn->find_property(L"ReturnValue");
    if (!ok || ret == nullptr) return nullptr;
    static bool s_said = false;
    if (!s_said) {
        s_said = true;
        for (API::FField* f = fn->get_child_properties(); f != nullptr; f = f->get_next()) {
            const auto* nm = f->get_fname();
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE param %ls @0x%X", nm ? nm->to_string().c_str() : L"?",
                                 (unsigned)static_cast<API::FProperty*>(f)->get_offset());
        }
    }
    fn->call(animbp, p);
    return *reinterpret_cast<API::UObject**>(p + ret->get_offset());
}
void sm_set_position(API::UObject* animbp, API::UObject* mont, float t) {
    auto* fn = animbp->get_class()->find_function(L"Montage_SetPosition");
    if (fn == nullptr) return;
    alignas(16) uint8_t p[0x40] = {0};
    bool ok = true;
    sm_put(fn, p, sizeof(p), L"Montage", &mont, sizeof(void*), &ok);
    sm_put(fn, p, sizeof(p), L"NewPosition", &t, sizeof(float), &ok);
    if (ok) fn->call(animbp, p);
}
void sm_stop(API::UObject* animbp, API::UObject* mont) {
    auto* fn = animbp->get_class()->find_function(L"Montage_Stop");
    if (fn == nullptr) return;
    alignas(16) uint8_t p[0x40] = {0};
    bool ok = true;
    const float bout = 0.05f;
    sm_put(fn, p, sizeof(p), L"InBlendOutTime", &bout, sizeof(float), &ok);
    sm_put(fn, p, sizeof(p), L"Montage", &mont, sizeof(void*), &ok);
    if (ok) fn->call(animbp, p);
}
void slide_montage_tick() {
    auto* animbp = reload_weapon_anim_instance();
    const bool usable = g_cfg.slide_montage && reload_rack_available() && animbp != nullptr && slide_weapon_ok();
    if (!usable) {
        if (s_sm_active) {
            if (auto* inst = s_sm_inst.get()) if (auto* m = s_sm_mont.get()) sm_stop(inst, m);
            s_sm_active = false; s_sm_mont = TrackedObject{};
            if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: stopped (unavailable)");
        }
        return;
    }
    sm_list_slots(animbp);
    const std::string spec = trim_cfg(g_cfg.slide_seq);
    if (spec.empty()) return;
    if (spec != s_sm_seq_last || s_sm_inst.get() != animbp) {
        s_sm_seq_last = spec; s_sm_inst.set(animbp);
        s_sm_active = false; s_sm_mont = TrackedObject{}; s_sm_seq = TrackedObject{}; s_sm_len = 0.0f;
        auto* seq = find_anim_sequence(spec);
        if (seq == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: no AnimSequence matching '%s'", spec.c_str()); return; }
        s_sm_seq.set(seq);
        if (auto* pl = seq->get_property_data<float>(L"SequenceLength")) if (!IsBadReadPtr(pl, sizeof(float))) s_sm_len = *pl;
        API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: sequence %ls, length %.3f s", seq->get_full_name().c_str(), s_sm_len);
    }
    auto* seq = s_sm_seq.get();
    if (seq == nullptr) return;
    const float pull = g_slide_pull.load(std::memory_order_relaxed);
    const float frac = (g_cfg.slide_travel > 0.0f) ? std::fmin(1.0f, std::fmax(0.0f, pull / g_cfg.slide_travel)) : 0.0f;
    const bool sweep = g_cfg.slide_seq_sweep > 0.0f && s_sm_len > 0.0f;
    const bool want = sweep || s_sl_held || pull > 0.0005f;
    float t = g_cfg.slide_seq_fwd + (g_cfg.slide_seq_back - g_cfg.slide_seq_fwd) * frac;
    if (sweep) {
        static long long s_t0 = 0;
        if (s_t0 == 0) s_t0 = now_ticks();
        const double sec = (double)(now_ticks() - s_t0) / (double)ms_to_ticks(1000);
        t = (float)std::fmod(sec / (double)g_cfg.slide_seq_sweep, 1.0) * s_sm_len;
    }
    if (want) {
        if (!s_sm_active || s_sm_mont.get() == nullptr) {
            auto* m = sm_play(animbp, seq, t);
            s_sm_active = (m != nullptr);
            if (m != nullptr) s_sm_mont.set(m);
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: play in slot '%s' at t=%.3f -> montage=%s",
                                 trim_cfg(g_cfg.slide_slot).c_str(), t, m ? "ok" : "NULL");
        } else {
            sm_set_position(animbp, s_sm_mont.get(), t);
        }
        if (sweep && g_cfg.slide_log) {
            static long long s_said = 0;
            if (now_ticks() - s_said > ms_to_ticks(250)) { s_said = now_ticks(); API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE sweep t=%.3f of %.3f", t, s_sm_len); }
        }
    } else if (s_sm_active) {
        if (auto* m = s_sm_mont.get()) sm_stop(animbp, m);
        s_sm_active = false; s_sm_mont = TrackedObject{};
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEMONTAGE: stopped (slide home)");
    }
}

API::UObject* native_mag_mesh_impl() {
    static std::string s_key; static TrackedObject s_mesh; static bool s_none = false; static long long s_at = 0;
    const std::string key = weapon_key();
    if (key != s_key || (s_mesh.get() == nullptr && !s_none) || (s_none && now_ticks() - s_at > ms_to_ticks(2000))) {
        s_key = key; s_mesh = TrackedObject{}; s_none = false; s_at = now_ticks();
        auto* src = reload_weapon_default_comp();
        if (src != nullptr) {
            weapon_components([&](API::UObject* c) {
                auto** pp = c->get_property_data<API::UObject*>(L"AttachParent");
                if (pp == nullptr || IsBadReadPtr(pp, sizeof(void*)) || *pp != src) return true;
                auto** pm = c->get_property_data<API::UObject*>(L"StaticMesh");
                if (pm == nullptr || IsBadReadPtr(pm, sizeof(void*)) || *pm == nullptr || (*pm)->get_fname() == nullptr) return true;
                std::wstring lo = (*pm)->get_fname()->to_string(); for (auto& ch : lo) ch = (wchar_t)towlower(ch);
                if (lo.find(L"magazine") == std::wstring::npos && lo.find(L"megazine") == std::wstring::npos) return true;
                s_mesh.set(*pm);
                if (g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD native magazine mesh for %s: %ls", key.c_str(), (*pm)->get_full_name().c_str());
                return false;
            });
        }
        if (s_mesh.get() == nullptr) s_none = true;
    }
    return s_mesh.get();
}

// ---- THE MAGAZINE ASSET BY NAME (reloadmagasset, doctrine in ConfigFields.inl). The component
// path above needs the weapon's magazine component to be attached to the Default skeletal mesh at
// the tick it runs, and on the magnum it was not. The marker renders an ASSET, not a component,
// so the asset can be taken straight off the loaded-object list by the name the weapon key
// implies: FP_Magnum -> stem "magnum" -> SM_magnum_magazine*, prefix-matched so every shipped
// variant lands. The names are the owner's own SLIDEPART native listings, not a guess:
//   SM_Magnum_Magazine_Default             FP_Magnum
//   SM_AssaultRifle_Magazine_M_Default     FP_AssaultRifle
//   SM_SMG_Magazine1Jnt_L_Default          FP_SMG   (no separator after Magazine: prefix, not exact)
//   SM_RocketLauncher_Magazine_M_Default   FP_RocketLauncher
// plus the classic assault rifle's asset spelled "Megazine", which mag_hide_apply and the rack's
// part scan already both treat as a magazine spelling. The shotgun's listing has no magazine part
// at all (HandleJnt, PumpJnt, ShellJnt, ShellSliderJnt, Glass_HandleJnt -- it loads shells), so it
// resolves to nothing and nothing is drawn, which is the answer the owner asked for in place of a
// wrong mesh.
//
// PACED. The walk is the whole ~290k object array (65-100 ms, measured by the holster's own
// survey), so: one walk when the weapon key changes, and for a weapon that resolves to nothing at
// most two more, no oftener than every 2 s. The name is read BEFORE the class name so only the
// sm_* objects pay for a second string. A miss is never stored as the marker's mesh.
API::UObject* mag_asset_by_name_impl() {
    static std::string s_key; static TrackedObject s_mesh; static bool s_none = false;
    static long long s_at = 0; static int s_walks = 0;
    const std::string key = weapon_key();
    if (key.empty()) return nullptr;
    const bool fresh_key = (key != s_key);
    const bool retry = s_none && s_walks < 3 && now_ticks() - s_at > ms_to_ticks(2000);
    if (fresh_key || (s_mesh.get() == nullptr && !s_none) || retry) {
        if (fresh_key) { s_key = key; s_walks = 0; }
        s_mesh = TrackedObject{}; s_none = false; s_at = now_ticks(); ++s_walks;
        std::wstring stem;
        {
            std::string k = key;
            if (k.rfind("FP_", 0) == 0) k.erase(0, 3);
            for (char ch : k) stem.push_back((wchar_t)towlower((unsigned char)ch));
        }
        auto* arr = (stem.empty()) ? nullptr : API::get()->get_uobject_array();
        if (arr != nullptr) {
            const std::wstring want_mag = L"sm_" + stem + L"_magazine";
            const std::wstring want_meg = L"sm_" + stem + L"_megazine";
            const int32_t nn = arr->get_object_count();
            for (int32_t i = 0; i < nn; ++i) {
                auto* o = static_cast<API::UObject*>(arr->get_object(i));
                if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
                const auto* fn = o->get_fname(); if (fn == nullptr) continue;
                std::wstring nm = fn->to_string(); for (auto& ch : nm) ch = (wchar_t)towlower(ch);
                if (nm.rfind(want_mag, 0) != 0 && nm.rfind(want_meg, 0) != 0) continue;
                if (nm.find(L"shadow") != std::wstring::npos) continue;   // the shadow proxy, not the part
                if (nm.find(L"_ui_") != std::wstring::npos) continue;     // the readout's copy
                if (class_name_of(o) != L"StaticMesh") continue;
                s_mesh.set_at(o, i);
                if (g_cfg.reload_vr_log)
                    API::get()->log_info("[Halo-CampE-UEVR] RELOAD magazine asset by name for %s: %ls (walk %d)",
                                         key.c_str(), o->get_full_name().c_str(), s_walks);
                break;
            }
        }
        if (s_mesh.get() == nullptr) {
            s_none = true;
            if (g_cfg.reload_vr_log)
                API::get()->log_info("[Halo-CampE-UEVR] RELOAD no SM_%ls_Magazine* asset loaded for %s (walk %d): NO magazine is drawn",
                                     stem.c_str(), key.c_str(), s_walks);
        }
    }
    return s_mesh.get();
}

// ---- ANIMSTATE (under slidelog): every change of the weapon AnimBP's FirstPersonState, so the
// fire state's enum value is read off a shot rather than guessed.
void anim_state_probe_tick() {
    if (!g_cfg.slide_log) return;
    auto* animbp = reload_weapon_anim_instance();
    if (animbp == nullptr) return;
    auto* p = animbp->get_property_data<uint8_t>(L"FirstPersonState");
    if (p == nullptr || IsBadReadPtr(p, 1)) return;
    static int s_last = -1, s_lines = 0;
    if ((int)*p != s_last && s_lines < 80) {
        ++s_lines;
        int tog = -1;
        if (auto* t = animbp->get_property_data<uint8_t>(L"AnimToggle")) if (!IsBadReadPtr(t, 1)) tog = (int)*t;
        API::get()->log_info("[Halo-CampE-UEVR] ANIMSTATE FirstPersonState %d -> %d (AnimToggle=%d)", s_last, (int)*p, tog);
        s_last = (int)*p;
    }
}

// ---- SLIDEFIRE. The first-person slide is posed only by the weapon's AnimBP, which has no Slot
// node, so the rack borrows the ABP's own FIRE state: FirstPersonState is held at the fire value
// and the mesh's GlobalAnimRateScale is steered every tick so the fire animation's time follows
// the hand (0 = slide home, slide_fire_back = fully back), then runs to the end on release. Time
// is estimated by integrating the rate we set against the tick's dt -- the ABP exposes no clock.
float s_gest_dt = 1.0f / 60.0f;
bool   s_sf_active = false, s_sf_releasing = false;
double s_sf_t = 0.0;
float  s_sf_len = 0.8f;
TrackedObject s_sf_comp, s_sf_inst;
uint8_t       s_sf_prev_state = 0;
void slide_fire_restore() {
    if (auto* c = s_sf_comp.get()) {
        if (auto* r = c->get_property_data<float>(L"GlobalAnimRateScale")) if (!IsBadWritePtr(r, sizeof(float))) *r = 1.0f;
    }
    // Hand the state back NOW: leaving "fire" set until the game's next 300 ms re-assertion
    // lets the fire animation restart at full rate in between (the second slam, 2026-09-03).
    if (s_sf_active) {
        if (auto* animbp = s_sf_inst.get()) {
            if (auto* st = animbp->get_property_data<uint8_t>(L"FirstPersonState")) if (!IsBadWritePtr(st, 1)) *st = s_sf_prev_state;
        }
    }
    s_sf_active = false; s_sf_releasing = false; s_sf_t = 0.0;
}
void slide_fire_tick() {
    auto* animbp = reload_weapon_anim_instance();
    auto* comp = reload_weapon_default_comp();
    const bool usable = g_cfg.slide_fire && !g_cfg.slide_copy && reload_rack_available() && animbp != nullptr && comp != nullptr && slide_weapon_ok()
                        && s_reload == ReloadState::Idle;
    if (!usable) { if (s_sf_active) { slide_fire_restore(); if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE: stopped (unavailable)"); } return; }
    auto* state = animbp->get_property_data<uint8_t>(L"FirstPersonState");
    auto* rate  = comp->get_property_data<float>(L"GlobalAnimRateScale");
    if (state == nullptr || rate == nullptr || IsBadWritePtr(state, 1) || IsBadWritePtr(rate, sizeof(float))) return;
    if (s_sf_comp.get() != comp) {
        s_sf_comp.set(comp);
        // The fire sequence's length, from the ABP's own FirstPersonFire slot.
        s_sf_len = 0.8f;
        for (API::UStruct* st = animbp->get_class(); st != nullptr; st = st->get_super_struct()) {
            if (IsBadReadPtr(st, sizeof(void*))) break;
            bool found = false;
            for (API::FField* f = st->get_child_properties(); f != nullptr; f = f->get_next()) {
                if (IsBadReadPtr(f, sizeof(void*))) break;
                auto* fc = f->get_class(); const auto* fn = f->get_fname();
                if (fc == nullptr || fn == nullptr || fc->get_name() != L"ObjectProperty") continue;
                const std::wstring nm = fn->to_string();
                static const std::wstring want = L"FirstPersonFire";
                if (nm.size() < want.size() || nm.compare(nm.size() - want.size(), want.size(), want) != 0) continue;
                auto** pp = reinterpret_cast<API::UObject**>(reinterpret_cast<uint8_t*>(animbp) + static_cast<API::FProperty*>(f)->get_offset());
                if (IsBadReadPtr(pp, sizeof(void*)) || *pp == nullptr) continue;
                if (auto* pl = (*pp)->get_property_data<float>(L"SequenceLength")) if (!IsBadReadPtr(pl, sizeof(float)) && *pl > 0.05f) s_sf_len = *pl;
                found = true; break;
            }
            if (found) break;
        }
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE: fire sequence length %.3f s, state value %d", s_sf_len, g_cfg.slide_fire_state);
    }
    const float pull = g_slide_pull.load(std::memory_order_relaxed);
    const float frac = (g_cfg.slide_travel > 0.0f) ? std::fmin(1.0f, std::fmax(0.0f, pull / g_cfg.slide_travel)) : 0.0f;
    const bool want = s_sl_held || pull > 0.0005f;
    const float dt = (s_gest_dt > 0.001f && s_gest_dt < 0.2f) ? s_gest_dt : (1.0f / 60.0f);
    if (!s_sf_active) {
        if (!want) return;
        s_sf_active = true; s_sf_releasing = false; s_sf_t = 0.0;
        s_sf_inst.set(animbp);
        s_sf_prev_state = *state;
        *state = (uint8_t)g_cfg.slide_fire_state;
        const bool entry = g_cfg.slide_fire_entry > 0.0f;
        *rate = entry ? 1.0f : 0.0f;   // the entry window (if any): let the transition blend in at normal rate
        if (entry) s_sf_t += (double)dt;
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE: enter state %d", g_cfg.slide_fire_state);
        return;
    }
    *state = (uint8_t)g_cfg.slide_fire_state;   // the game re-asserts its own each tick; hold ours
    if (!want && !s_sf_releasing) { s_sf_releasing = true; if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE: released at t=%.3f, running to the end", s_sf_t); }
    float r = 0.0f;
    if (s_sf_releasing) {
        r = 1.0f;
    } else if (s_sf_t < (double)g_cfg.slide_fire_entry) {
        r = 1.0f;   // still inside the entry window
    } else {
        const double target = std::fmax((double)g_cfg.slide_fire_entry, (double)frac * (double)g_cfg.slide_fire_back);
        if (s_sf_t < target) r = (float)std::fmin(3.0, (target - s_sf_t) / (double)dt);
    }
    if (g_cfg.slide_log) {
        static long long s_said = 0;
        if (now_ticks() - s_said > ms_to_ticks(250)) { s_said = now_ticks(); API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE t=%.3f rate=%.2f frac=%.2f state=%d", s_sf_t, r, frac, (int)*state); }
    }
    *rate = r;
    s_sf_t += (double)r * (double)dt;
    // Home again: hand the state back here, before the recoil and before the game's next idle
    // re-assertion can restart the fire at full rate.
    const double stop_at = (g_cfg.slide_fire_fwd > 0.0f) ? std::fmin((double)g_cfg.slide_fire_fwd, (double)s_sf_len) : (double)s_sf_len;
    if (s_sf_releasing && s_sf_t >= stop_at) {
        slide_fire_restore();
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEFIRE: home at t=%.3f, state handed back", s_sf_t);
    }
}

// ---- SLIDECHAMBER: the reload a seated mag is waiting for fires once the rack has completed and
// the fire-state slide has snapped home (so the game's reload animation, which the state hold
// then suppresses, never overlaps the rack's own motion).
uint16_t* rounds_field() {
    // The object pointer is resolved on the sim thread from the published datum, so right after a
    // swap it can still be the previous weapon's: no read or write lands on that gun.
    const int32_t held = g_wpn_obj_index.load(std::memory_order_relaxed);
    if (held != -1 && g_wpn_obj_ptr_datum.load(std::memory_order_relaxed) != held) return nullptr;
    const uintptr_t obj = g_wpn_obj_ptr.load(std::memory_order_relaxed);
    const int off = g_cfg.rounds_off;
    if (obj == 0 || off <= 0 || off > 0x7FE) return nullptr;
    auto* r = reinterpret_cast<uint16_t*>(obj + (uintptr_t)off);
    if (IsBadWritePtr(r, sizeof(uint16_t))) return nullptr;
    return r;
}
// Is the gun in hand empty right now? The rounds counter when it reads (the phantom's one round
// counts as empty), else the AnimBP's two-state ammunition frame. The frame alone reads 0 on a
// freshly spawned weapon until its first reload (logged: 14 rounds, frame 0), which dropped a loaded
// rifle as empty: trigger dead with the mag out, a rack forced, the game's reload pressed at the drop.
bool weapon_empty_now() {
    if (s_true_empty) return true;
    if (auto* r = rounds_field()) return *r == 0;
    return slide_ammo_frame() == 0;
}
// ---- SLIDEPHANTOM: see Config.hpp. Runs every tick on the game thread against the resolved
// weapon object (ordinary heap memory once resolved).
// The session kind: standalone (single player) or networked (coop, host or client).
bool net_is_coop() {
    static long long s_at = 0; static bool s_coop = false; static bool s_said = false;
    const long long nowt = now_ticks();
    if (nowt - s_at < ms_to_ticks(2000)) return s_coop;
    s_at = nowt;
    auto* pawn = API::get()->get_local_pawn(0);
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.KismetSystemLibrary");
    auto* cdo = cls ? cls->get_class_default_object() : nullptr;
    if (pawn == nullptr || cdo == nullptr) return s_coop;
    alignas(16) uint8_t p[64] = {0};
    *reinterpret_cast<void**>(p) = pawn;
    cdo->call_function(L"IsStandalone", p);
    const bool standalone = p[8] != 0;
    const bool coop = !standalone;
    bool server = false;
    { alignas(16) uint8_t q[64] = {0}; *reinterpret_cast<void**>(q) = pawn; cdo->call_function(L"IsServer", q); server = q[8] != 0; }
    if (coop != s_coop || !s_said) { s_said = true; API::get()->log_info("[Halo-CampE-UEVR] NET: session is %s, this machine is the %s%s", coop ? "NETWORKED (coop)" : "standalone", server ? "HOST" : "CLIENT", (coop && g_cfg.coop_auto) ? (g_cfg.coop_hide ? " -- hidden reload: the host reloads underneath, frozen and locked until our gesture" : " -- phantom round and auto-reload undo stand down") : ""); }
    s_coop = coop;
    return s_coop;
}
// The hidden reload applies in coop (reloadcoophide) and, with reloadhidesolo, in solo too.
bool reload_hidden_mode() {
    if (!g_cfg.coop_hide) return false;
    if (g_cfg.hide_solo) return true;
    return g_cfg.coop_auto && net_is_coop();
}
void ad_write_zero(bool render_path);   // the anim-variable readout probe (defined with the display block below)
// The phantom's per-weapon flags live outside the tick so the per-weapon reload state carries them.
// The displays read 0 from the dry shot until the mag is SEATED (not the drop, where the press
// clears the lock): s_hide_display outlives s_true_empty through the gesture.
bool    s_hide_display = false;
int     s_coop_lock_rounds = 1;
int     s_ph_reserve_seen = -1;   // the reserve offset the phantom tick is using (reloadshotgunlog)
int     s_rt_taps = 0, s_rt_presses = 0;   // taps the engine saw and presses it sent (reloadshotgunlog)
bool    s_ph_rebase = false;      // the weapon in hand changed identity: re-seed the counter caches
int32_t s_rs_cur_datum = -1;      // the held weapon's datum read this tick by the reload state tracker
void slide_phantom_tick() {
    static std::string s_key;
    static int s_prev = -1;
    static uint16_t s_snap[0x400]; static bool s_have_snap = false;   // last tick's first 0x800 bytes, for the reserve search
    static int s_reserve = -1;
    if (g_cfg.slide_phantom == 0 && !g_cfg.slide_undo_reload) {
        // The hidden reload's display hide is released below this return; switched off mid-lock it
        // stayed set and the ammo displays read 0 on a loaded gun.
        s_true_empty = false; s_hide_display = false; g_wristhud_hide_cradle.store(false, std::memory_order_relaxed);
        s_prev = -1; s_have_snap = false; return;
    }
    const bool coop = g_cfg.coop_auto && net_is_coop();
    const bool hidden = reload_hidden_mode();
    if (!hidden || (s_reload == ReloadState::Idle && !s_true_empty && !s_sl_pressed_early)) s_hide_display = false;
    g_wristhud_hide_cradle.store(hidden && (s_true_empty || s_hide_display), std::memory_order_relaxed);
    if (hidden && (s_true_empty || s_hide_display)) ad_write_zero(false);
    if (coop && !g_cfg.coop_hide && g_cfg.coop_stop_at <= 0) { s_true_empty = false; s_prev = -1; s_have_snap = false; return; }
    const std::string key = weapon_key();
    static long long s_key_at = 0, s_shot_at = 0;
    // Tracked only while the tracker runs (it lives in reload_update, behind reloadvr): with it
    // stopped nothing saves or loads the flags on a swap, and its datum goes stale.
    const bool tracked = g_cfg.reload_state_id != 0 && reload_manual_available();
    if (key != s_key || (tracked && s_ph_rebase)) {
        // Tracked: the flags already belong to the weapon now in hand (the tracker saved the old
        // weapon's and loaded this one's earlier in this tick); only the counter caches re-seed.
        // Dropping the flag here is what handed a gun back its full magazine for free.
        if (s_true_empty && g_cfg.slide_log) API::get()->log_info(tracked ? "[Halo-CampE-UEVR] PHANTOM: weapon changed; the flag stays with its weapon's record"
                                                                          : "[Halo-CampE-UEVR] PHANTOM: weapon changed while truly empty; flag dropped");
        s_key = key; s_key_at = now_ticks(); s_prev = -1; s_have_snap = false; s_reserve = (g_cfg.reserve_off >= 0) ? g_cfg.reserve_off : -1;
        if (!tracked) { s_true_empty = false; s_hide_display = false; }
        s_ph_rebase = false;
    }
    auto* r = rounds_field();
    if (r == nullptr) { s_prev = -1; s_have_snap = false; return; }
    // Tracked: right after a swap the object pointer can still be the previous weapon's (it is
    // resolved on the sim thread from a published datum); reading or writing that counter would
    // judge -- or clear -- this weapon's flags on the wrong gun.
    if (tracked && s_rs_cur_datum != -1 && g_wpn_obj_ptr_datum.load(std::memory_order_relaxed) != s_rs_cur_datum) { s_prev = -1; s_have_snap = false; return; }
    const int cur = (int)*r;
    const uintptr_t obj = g_wpn_obj_ptr.load(std::memory_order_relaxed);
    const bool obj_ok = obj != 0 && !IsBadReadPtr((const void*)obj, 0x800);
    // A REFILL THE PLAYER DID NOT ASK FOR: rounds jumped up while no reload of ours was in flight.
    // The reserve counter is the u16 that dropped by the same amount in the same tick.
    if (s_prev >= 0 && cur > s_prev + 1 && obj_ok && s_have_snap) {
        const int refill = cur - s_prev;
        const uint16_t* now16 = reinterpret_cast<const uint16_t*>(obj);
        int found = -1, nfound = 0;
        for (int i = 0; i < 0x400; ++i) {
            if (i * 2 == g_cfg.rounds_off) continue;
            if ((int)s_snap[i] - (int)now16[i] == refill && s_snap[i] < 4000) { if (found < 0) found = i * 2; ++nfound; }
        }
        if (found >= 0 && s_reserve < 0) { s_reserve = found; API::get()->log_info("[Halo-CampE-UEVR] PHANTOM: reserve counter found at +0x%03X (dropped by %d with the refill; %d candidate(s))", (unsigned)found, refill, nfound); }
        const bool ours = s_sl_reload_due || s_sl_pressed_early
                       || (g_reload_hold_until.load(std::memory_order_relaxed) > now_ticks() - ms_to_ticks(2000))
                       || (s_sl_press_at != 0 && now_ticks() - s_sl_press_at < ms_to_ticks(g_cfg.reload_mute_ms + 1500));
        if (!coop && !hidden && g_cfg.slide_undo_reload && s_true_empty && !ours) {
            *r = 0;
            if (s_reserve >= 0 && s_reserve < 0x7FE) {
                auto* rs = reinterpret_cast<uint16_t*>(obj + (uintptr_t)s_reserve);
                if (!IsBadWritePtr(rs, 2)) { const int v = (int)*rs + refill; *rs = (uint16_t)(v > 65535 ? 65535 : v); }
            }
            reload_state_hold_begin();   // the game's reload animation stays off screen
            API::get()->log_info("[Halo-CampE-UEVR] PHANTOM: undid a game auto-reload of %d rounds (reserve %s); gun stays empty", refill, s_reserve >= 0 ? "restored" : "UNKNOWN, not restored");
            s_prev = 0;
            if (obj_ok) { memcpy(s_snap, reinterpret_cast<const void*>(obj), sizeof(s_snap)); s_have_snap = true; }
            return;
        }
    }
    // NEVER an auto-reload (from the headset, 2026-09-06): the lock has no timeout. With no reserve the
    // last round simply stays locked, which costs one round and never a zero. The searched reserve
    // offset is not trusted here (in coop it picked a field that reads 0 with a full mag); only a
    // cfg-given reserve offset may veto the lock.
    bool reserve_empty = false;
    if (coop && obj_ok && g_cfg.reserve_off >= 0 && s_reserve >= 0 && s_reserve < 0x7FE) { const auto* rs = reinterpret_cast<const uint16_t*>(obj + (uintptr_t)s_reserve); if (!IsBadReadPtr(rs, 2) && *rs == 0) reserve_empty = true; }
    // FULL AUTO (2026-09-06 21:43, one shot got out 84 ms after the lock): between the tick that
    // sees the count and the sim seeing the trigger let go there is room for one more shot at the
    // AR's rate, so while shots are landing at that pace the lock goes on one round early. The
    // worst case then ends on one, never zero. The key must be 500 ms old: the counter reads junk
    // for a few ticks around a weapon swap.
    const long long nowt_c = now_ticks();
    const bool firing_fast = nowt_c - s_shot_at < ms_to_ticks(250);
    const int stop_at = g_cfg.coop_stop_at + (firing_fast ? 1 : 0);
    // A REAL last shot only: exactly one round going to zero on a weapon held two seconds. A
    // weapon's counter reads 0 while it initialises and on a swap to a plasma weapon (60 -> 0 in
    // the log), and the first build fired the dry trigger in the mission's first second and froze
    // the pose under a live gun (the climbing pitch, 2026-09-07).
    if (reload_manual_available() && hidden && !s_true_empty && s_prev == 1 && cur == 0 && nowt_c - s_key_at > ms_to_ticks(2000)
        && s_reload == ReloadState::Idle && !s_sl_reload_due && !s_sl_pressed_early && !weapon_in_list(g_cfg.reload_skip_weapons)) {
        // The last round went out and the game is reloading: freeze the pose, mute it, show empty,
        // dead trigger. Nothing clears this but our own reload (reload_press_now) or a swap.
        s_true_empty = true; s_hide_display = true; s_coop_lock_rounds = 0x7FFF;
        ak_mute_begin();
        reload_state_hold_begin();
        reload_anim_rate_begin();
        if (g_cfg.coop_mask_ms > 0) reload_pose_hold(g_cfg.coop_mask_ms);   // the hands too, until our gesture
        if (g_cfg.slide_log || g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] DRY (%s): last round fired, the game reloads underneath; pose frozen, muted, locked until our reload", coop ? "coop" : "solo");
        s_prev = cur;
        if (obj_ok) { memcpy(s_snap, reinterpret_cast<const void*>(obj), sizeof(s_snap)); s_have_snap = true; }
        return;
    }
    if (reload_manual_available() && coop && !g_cfg.coop_hide && !s_true_empty && cur >= 1 && cur <= stop_at && !reserve_empty && nowt_c - s_key_at > ms_to_ticks(500)
        && s_reload == ReloadState::Idle && !s_sl_reload_due && !s_sl_pressed_early
        && !weapon_in_list(g_cfg.reload_skip_weapons)) {   // every weapon with a magazine, rack or not (the SMG has no rack part)
        // The dry stop: no write, the last round stays in the counter and the trigger is dead.
        s_true_empty = true; s_coop_lock_rounds = cur;
        if (g_cfg.slide_log || g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] COOP dry stop: %d round(s) left, trigger locked and slide back until our reload", cur);
        s_prev = cur;
        if (obj_ok) { memcpy(s_snap, reinterpret_cast<const void*>(obj), sizeof(s_snap)); s_have_snap = true; }
        return;
    }
    if (reload_manual_available() && !coop && !hidden && !s_true_empty && s_prev == 1 && cur == 0 && slide_weapon_ok() && slide_chamber_ok() && slide_rack_available() && g_cfg.slide_phantom > 0) {
        *r = 1;
        s_true_empty = true;
        if (g_cfg.slide_log || g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] PHANTOM: magazine empty; one phantom round held so the game does not auto-reload (trigger dead, slide back)");
        s_prev = 1;
        if (obj_ok) { memcpy(s_snap, reinterpret_cast<const void*>(obj), sizeof(s_snap)); s_have_snap = true; }
        return;
    }
    if (s_true_empty) {
        if (cur > ((coop || hidden) ? s_coop_lock_rounds : 1) || (coop && reserve_empty)) {
            s_true_empty = false;
            if (g_cfg.slide_log || g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] PHANTOM: rounds now %d, the gun reloaded; phantom cleared", cur);
        } else {
            if (!coop && !hidden && g_cfg.slide_phantom == 2 && cur == 0) *r = 1;   // re-asserted: the sim never sees 0
            if (auto* animbp = reload_weapon_anim_instance()) {
                if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame")) if (!IsBadWritePtr(p, sizeof(int32_t))) *p = 0;
            }
        }
    }
    // A PUMP BETWEEN SHOTS (@everyshot): a shot fired outside our reload locks the gun until the
    // rack completes; the rack is live while the lock waits, hold or no hold.
    if (g_sl_zone_every_shot.load(std::memory_order_relaxed) && cur < s_prev && s_prev > 0 && !s_sl_lock_pending && s_reload == ReloadState::Idle) {
        s_sl_lock_pending = true; s_sl_lock_frame = 0; s_sl_rack_done = false;
        if (g_cfg.slide_log || g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDE shot fired on a pump weapon: locked until the rack");
    }
    {
        const int fin = (int)*r;
        if (s_prev >= 0 && fin > s_prev + 1) g_last_refill_at = now_ticks();
        if (s_prev >= 0 && fin < s_prev && s_prev - fin <= 3) s_shot_at = now_ticks();
        if (g_cfg.reload_vr_log && s_prev >= 0 && fin != s_prev) {
            int reserve = -1;
            if (obj_ok && s_reserve >= 0 && s_reserve < 0x7FE) { const auto* rs = reinterpret_cast<const uint16_t*>(obj + (uintptr_t)s_reserve); if (!IsBadReadPtr(rs, 2)) reserve = (int)*rs; }
            API::get()->log_info("[Halo-CampE-UEVR] AMMO rounds %d -> %d (reserve %d)%s", s_prev, fin, reserve, fin > s_prev ? "  <-- REFILL" : "");
        }
        s_prev = fin;
    }
    if (obj_ok) { memcpy(s_snap, reinterpret_cast<const void*>(obj), sizeof(s_snap)); s_have_snap = true; }
    s_ph_reserve_seen = s_reserve;
}

// The game's reload press with everything that rides it: the honest 0 first so the refill
// accounts from empty, the hold, the animation clamp, the sound mute.
void reload_press_now(const char* why) {
    const long long nowt = now_ticks();
    ++s_rt_presses;
    if (s_true_empty) {
        s_true_empty = false;   // stop the every-tick re-assert BEFORE the 0 goes back, or the refill counts from 1
        if (!(g_cfg.coop_auto && net_is_coop()) && !reload_hidden_mode()) { if (auto* r = rounds_field()) { if (*r == 1) *r = 0; } }   // coop / hidden: the counter is never written
        if (g_cfg.slide_log || g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] PHANTOM: rounds set back to 0 for the reload (%s)", why);
    }
    g_reload_hold_until.store(nowt + ms_to_ticks(net_is_coop() ? g_cfg.reload_press_ms_coop : g_cfg.reload_press_ms), std::memory_order_relaxed);
    s_sl_press_at = nowt;
    if (reload_hidden_mode()) reload_pose_hold(g_cfg.reload_mask_ms);   // the hidden reload's hand hold ends here (0 releases)
    else if (g_cfg.reload_mask_ms > 0) reload_pose_hold(g_cfg.reload_mask_ms);
    reload_anim_rate_begin();
    reload_state_hold_begin();
    ak_mute_begin();
    if (g_cfg.reload_vr_log || g_cfg.slide_log) { const auto* r = rounds_field(); API::get()->log_info("[Halo-CampE-UEVR] RELOAD pressed (%s): %d ms, rounds %d", why, net_is_coop() ? g_cfg.reload_press_ms_coop : g_cfg.reload_press_ms, r ? (int)*r : -1); }
}
bool reload_gestures_busy() { return s_reload != ReloadState::Idle || s_sl_lock_pending || s_sl_reload_due; }
void slide_chamber_tick() {
    if (s_sl_press_due_at != 0 && now_ticks() >= s_sl_press_due_at) {
        s_sl_press_due_at = 0;
        if (s_reload != ReloadState::Idle) { s_sl_pressed_early = true; reload_press_now("chambered round fired"); }
    }
    if (!s_sl_reload_due) return;
    if (s_reload != ReloadState::Idle || !slide_weapon_ok() || !slide_chamber_ok()) { s_sl_reload_due = false; return; }
    if (s_sl_lock_pending || s_sf_active) return;   // not racked yet, or the slide is still moving
    if (s_sl_pressed_early) { s_sl_pressed_early = false; s_sl_reload_due = false; if (g_cfg.reload_vr_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD rack complete, the sim reloaded at the drop"); return; }
    reload_press_now("rack");
    s_sl_reload_due = false;
}

// The engine's destroy is K2_DestroyComponent(Object) in reflection; a call to "DestroyComponent"
// finds no function and silently leaves the component alive (2026-09-04: every re-spawn stacked
// another gun).
void ue_destroy_component(API::UObject* c) {
    if (c == nullptr || IsBadReadPtr(c, sizeof(void*))) return;
    alignas(16) uint8_t q[64] = {0}; *reinterpret_cast<void**>(q) = c;
    c->call_function(L"K2_DestroyComponent", q);
}
// ---- SLIDECOPY. See Config.hpp. Every engine call goes through the UFunction's own parameter
// offsets (sm_put), never an assumed layout; FTransform is the UE5 double layout (rotation quat
// at 0, translation at 0x20, scale at 0x40, 0x60 bytes, 16-aligned).
struct alignas(16) XformD { double qx, qy, qz, qw; double tx, ty, tz; double pad0; double sx, sy, sz; double pad1; };
TrackedObject s_sc_copy, s_sc_src, s_sc_actor, s_sc_follower, s_sc_seq;
float         s_sc_len = 0.0f;
std::string   s_sc_key;
API::FName    s_sc_bone;
bool          s_sc_bone_ok = false;
bool          s_sc_hidden = false;
bool          s_sc_failed = false;
long long     s_sc_retry_at = 0;
uint32_t      s_sc_ticks = 0;
API::UFunction* sc_fn(API::UObject* o, const wchar_t* name) {
    if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) return nullptr;
    auto* c = o->get_class();
    return c ? c->find_function(name) : nullptr;
}
void sc_set_visibility(API::UObject* c, bool vis) {
    if (c == nullptr) return;
    alignas(16) uint8_t p[64] = {0}; p[0] = vis ? 1 : 0; p[1] = 0;
    c->call_function(L"SetVisibility", p);
}
void sc_set_scale(API::UObject* c, double sc) {
    if (c == nullptr) return;
    alignas(16) uint8_t p[64] = {0}; auto* d = reinterpret_cast<double*>(p); d[0] = sc; d[1] = sc; d[2] = sc;
    c->call_function(L"SetRelativeScale3D", p);
}
void sc_hide_real(API::UObject* src, bool hide) {
    if (src == nullptr) return;
    if (g_cfg.slide_copy_hide == 1) sc_set_visibility(src, !hide);
    else if (g_cfg.slide_copy_hide == 2) sc_set_scale(src, hide ? 0.001 : 1.0);
}
// The real component's render flags, read by name and mirrored onto the copy through the
// engine's own setters (a bare property write does not recreate the render proxy). Logged so
// the flag that makes a first-person primitive draw is a fact, not a guess.
void sc_mirror_flags(API::UObject* src, API::UObject* copy) {
    static const wchar_t* kBools[] = { L"bOnlyOwnerSee", L"bOwnerNoSee", L"bRenderInMainPass", L"bRenderInDepthPass",
                                       L"bVisibleInReflectionCaptures", L"bVisibleInRealTimeSkyCaptures", L"bVisibleInRayTracing",
                                       L"bReceivesDecals", L"bRenderCustomDepth", L"CastShadow", L"bVisible", L"bHiddenInGame",
                                       L"bTreatAsBackgroundForOcclusion", L"bUseAsOccluder", L"bCastInsetShadow", L"bSelfShadowOnly" };
    std::wstring line;
    for (const wchar_t* nm : kBools) {
        auto* pr = src->get_class()->find_property(nm);
        if (pr == nullptr) continue;
        auto* pcls = pr->get_class();
        if (pcls == nullptr || pcls->get_name() != L"BoolProperty") continue;
        const bool v = static_cast<API::FBoolProperty*>(pr)->get_value_from_object(src);
        line += std::wstring(nm) + L"=" + (v ? L"1 " : L"0 ");
    }
    // The 5.5 first-person primitive type, and any other byte/enum on the primitive with a name
    // that says first person.
    int fp_type = -1;
    for (API::UStruct* st = src->get_class(); st != nullptr; st = st->get_super_struct()) {
        if (IsBadReadPtr(st, sizeof(void*))) break;
        for (API::FField* f = st->get_child_properties(); f != nullptr; f = f->get_next()) {
            if (IsBadReadPtr(f, sizeof(void*))) break;
            auto* fc = f->get_class(); const auto* fn = f->get_fname();
            if (fc == nullptr || fn == nullptr) continue;
            const std::wstring cls = fc->get_name(), nm = fn->to_string();
            if (nm.find(L"FirstPerson") == std::wstring::npos) continue;
            const int32_t off = static_cast<API::FProperty*>(f)->get_offset();
            const uint8_t* q = reinterpret_cast<const uint8_t*>(src) + off;
            if (IsBadReadPtr(q, 4)) continue;
            int v = -1;
            if (cls == L"ByteProperty" || cls == L"EnumProperty" || cls == L"BoolProperty") v = (int)*q;
            else if (cls == L"IntProperty") v = *reinterpret_cast<const int32_t*>(q);
            else if (cls == L"FloatProperty") v = (int)(*reinterpret_cast<const float*>(q) * 1000.0f);
            line += nm + L"(" + cls + L")=" + std::to_wstring(v) + L" ";
            if (nm == L"FirstPersonPrimitiveType") fp_type = v;
        }
    }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: real mesh flags: %ls", line.c_str());
    // Nanite: the asset's setting and any Nanite-named flag on the copy, by name.
    {
        std::wstring nl;
        for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
            auto** pm = src->get_property_data<API::UObject*>(nm);
            if (pm == nullptr || IsBadReadPtr(pm, sizeof(void*)) || *pm == nullptr) continue;
            auto* asset = *pm;
            if (auto* pr = asset->get_class()->find_property(L"NaniteSettings")) {
                const uint8_t* q = reinterpret_cast<const uint8_t*>(asset) + pr->get_offset();
                if (!IsBadReadPtr(q, 1)) nl += L"asset NaniteSettings.bEnabled=" + std::to_wstring((int)(q[0] & 1)) + L" ";
            }
            break;
        }
        for (API::UStruct* st = copy->get_class(); st != nullptr; st = st->get_super_struct()) {
            if (IsBadReadPtr(st, sizeof(void*))) break;
            for (API::FField* f = st->get_child_properties(); f != nullptr; f = f->get_next()) {
                if (IsBadReadPtr(f, sizeof(void*))) break;
                auto* fc = f->get_class(); const auto* fn = f->get_fname();
                if (fc == nullptr || fn == nullptr) continue;
                const std::wstring nm = fn->to_string();
                if (nm.find(L"Nanite") == std::wstring::npos) continue;
                int v = -1;
                if (fc->get_name() == L"BoolProperty") v = static_cast<API::FBoolProperty*>(f)->get_value_from_object(copy) ? 1 : 0;
                nl += L"copy." + nm + L"(" + fc->get_name() + L")=" + std::to_wstring(v) + L" ";
            }
        }
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: nanite: %ls", nl.empty() ? L"(nothing named Nanite)" : nl.c_str());
    }
    fp_type = g_cfg.slide_copy_fp;   // the copy's type is a cfg choice, not a mirror (2026-09-04)
    if (fp_type >= 0) {
        if (auto* fn = sc_fn(copy, L"SetFirstPersonPrimitiveType")) {
            alignas(16) uint8_t p[64] = {0}; bool ok = true; const uint8_t t = (uint8_t)fp_type;
            for (API::FField* f = fn->get_child_properties(); f != nullptr; f = f->get_next()) {
                const auto* fnm = f->get_fname();
                if (fnm == nullptr) continue;
                const std::wstring nm = fnm->to_string();
                if (nm != L"ReturnValue") { sm_put(fn, p, sizeof(p), nm.c_str(), &t, 1, &ok); break; }
            }
            if (ok) { fn->call(copy, p); API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: copy FirstPersonPrimitiveType set to %d", fp_type); }
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: no SetFirstPersonPrimitiveType on the copy");
        }
    }
    auto mirror_bool = [&](const wchar_t* prop, const wchar_t* setter) {
        auto* pr = src->get_class()->find_property(prop);
        if (pr == nullptr || pr->get_class() == nullptr || pr->get_class()->get_name() != L"BoolProperty") return;
        const bool v = static_cast<API::FBoolProperty*>(pr)->get_value_from_object(src);
        alignas(16) uint8_t p[64] = {0}; p[0] = v ? 1 : 0;
        copy->call_function(setter, p);
    };
    mirror_bool(L"bOnlyOwnerSee", L"SetOnlyOwnerSee");
    mirror_bool(L"bOwnerNoSee", L"SetOwnerNoSee");
    // NOT mirrored: the real mesh has bRenderInMainPass=0 (measured 2026-09-04) and is drawn by
    // something else; a copy with the same flag draws nowhere. The copy stays in the main pass.
    { alignas(16) uint8_t p[64] = {0}; p[0] = 1; copy->call_function(L"SetRenderInMainPass", p); }
    mirror_bool(L"bRenderInDepthPass", L"SetRenderInDepthPass");
    mirror_bool(L"CastShadow", L"SetCastShadow");
    // A visibility toggle recreates the render proxy with the mirrored flags.
    sc_set_visibility(copy, false);
    sc_set_visibility(copy, true);
}
void sc_teardown(const char* why) {
    if (auto* f = s_sc_follower.get()) { alignas(16) uint8_t p[64] = {0}; ue_destroy_component(f); }
    s_sc_follower = TrackedObject{};
    if (auto* c = s_sc_copy.get()) { alignas(16) uint8_t p[64] = {0}; ue_destroy_component(c); }
    if (s_sc_hidden) { if (auto* src = s_sc_src.get()) { sc_set_visibility(src, true); sc_set_scale(src, 1.0); } }
    if (s_sc_copy.get() != nullptr || s_sc_hidden) {
        if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: torn down (%s)", why);
    }
    s_sc_copy = TrackedObject{}; s_sc_src = TrackedObject{}; s_sc_actor = TrackedObject{};
    s_sc_hidden = false; s_sc_bone_ok = false; s_sc_key.clear(); s_sc_ticks = 0;
}
// The real mesh's attach parent, socket and relative transform, read from its own properties.
bool sc_read_attach(API::UObject* src, API::UObject** parent, API::FName* socket, double loc[3], double rot[3], double scl[3]) {
    auto** pp = src->get_property_data<API::UObject*>(L"AttachParent");
    if (pp == nullptr || IsBadReadPtr(pp, sizeof(void*))) return false;
    *parent = *pp;
    auto* ps = src->get_property_data<API::FName>(L"AttachSocketName");
    if (ps == nullptr || IsBadReadPtr(ps, sizeof(int32_t) * 2)) return false;
    memcpy(socket, ps, sizeof(int32_t) * 2);
    auto* pl = src->get_property_data<double>(L"RelativeLocation");
    auto* pr = src->get_property_data<double>(L"RelativeRotation");
    auto* pc = src->get_property_data<double>(L"RelativeScale3D");
    if (pl == nullptr || pr == nullptr || pc == nullptr) return false;
    if (IsBadReadPtr(pl, 24) || IsBadReadPtr(pr, 24) || IsBadReadPtr(pc, 24)) return false;
    for (int i = 0; i < 3; ++i) { loc[i] = pl[i]; rot[i] = pr[i]; scl[i] = pc[i]; }
    return true;
}
void sc_dump_asset_users(API::UObject* asset);   // defined below
bool sc_spawn(API::UObject* actor, API::UObject* src) {
    const bool seq_mode = (g_cfg.slide_copy_mode == 1);
    auto* cls = API::get()->find_uobject<API::UClass>((seq_mode || g_cfg.slide_copy_class == 1) ? L"Class /Script/Engine.SkeletalMeshComponent"
                                                                                                : L"Class /Script/Engine.PoseableMeshComponent");
    if (cls == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: copy component class not found"); return false; }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: spawning a %ls", (seq_mode || g_cfg.slide_copy_class == 1) ? L"SkeletalMeshComponent" : L"PoseableMeshComponent");
    // The mesh asset, whichever name this engine version gives it.
    API::UObject* mesh = nullptr;
    for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh", L"SkeletalMeshAsset" }) {
        auto** pm = src->get_property_data<API::UObject*>(nm);
        if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
    }
    if (mesh == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: the real mesh has no skeletal mesh asset property I can read"); return false; }
    API::UObject* parent = nullptr; API::FName socket{}; double loc[3] = {0}, rot[3] = {0}, scl[3] = {1, 1, 1};
    if (!sc_read_attach(src, &parent, &socket, loc, rot, scl)) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: cannot read the real mesh's attachment"); return false; }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: real mesh %ls, parent %ls '%ls' socket '%ls', rel loc (%.2f %.2f %.2f) rot (%.2f %.2f %.2f) scale (%.2f %.2f %.2f)",
                         mesh->get_full_name().c_str(),
                         parent ? class_name_of(parent).c_str() : L"null",
                         (parent && parent->get_fname()) ? parent->get_fname()->to_string().c_str() : L"?",
                         socket.to_string().c_str(), loc[0], loc[1], loc[2], rot[0], rot[1], rot[2], scl[0], scl[1], scl[2]);
    auto* copy = API::get()->add_component_by_class(actor, cls, false);
    if (copy == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: add_component_by_class failed"); return false; }
    // Off the Nanite path BEFORE the mesh is set, so the render object is created classic.
    // Only the poseable: a SkeletalMeshComponent forced off Nanite has no classic data to draw on
    // a Nanite-only asset (2026-09-04: that is why the single-node copy vanished).
    if (g_cfg.slide_copy_no_nanite && !seq_mode && g_cfg.slide_copy_class == 0) {
        auto* pr = copy->get_class()->find_property(L"bForceDisableNanite");
        if (pr != nullptr && pr->get_class() != nullptr && pr->get_class()->get_name() == L"BoolProperty") {
            static_cast<API::FBoolProperty*>(pr)->set_value_in_object(copy, true);
            API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: bForceDisableNanite=1 on the copy");
        } else {
            API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: no bForceDisableNanite on the copy's class");
        }
    }
    // The mesh: 5.1+ names the setter SetSkinnedAssetAndUpdate; older engines SetSkeletalMesh.
    bool mesh_set = false;
    for (const wchar_t* fname : { L"SetSkinnedAssetAndUpdate", L"SetSkeletalMesh" }) {
        auto* fn = sc_fn(copy, fname);
        if (fn == nullptr) continue;
        alignas(16) uint8_t p[64] = {0}; bool ok = true; const bool reinit = true;
        sm_put(fn, p, sizeof(p), L"NewMesh", &mesh, sizeof(void*), &ok);
        sm_put(fn, p, sizeof(p), L"bReinitPose", &reinit, 1, &ok);
        if (!ok) continue;
        fn->call(copy, p); mesh_set = true;
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: mesh set via %ls", fname);
        break;
    }
    if (!mesh_set) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: no mesh setter found on the copy"); alignas(16) uint8_t p[64] = {0}; ue_destroy_component(copy); return false; }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 0; copy->call_function(L"SetCollisionEnabled", p); }
    if (parent != nullptr) {
        auto* fn = sc_fn(copy, L"K2_AttachToComponent");
        alignas(16) uint8_t p[128] = {0}; bool ok = true; const uint8_t rule = 0; const bool weld = false;
        if (fn != nullptr) {
            sm_put(fn, p, sizeof(p), L"Parent", &parent, sizeof(void*), &ok);
            sm_put(fn, p, sizeof(p), L"SocketName", &socket, sizeof(int32_t) * 2, &ok);
            sm_put(fn, p, sizeof(p), L"LocationRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"RotationRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"ScaleRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bWeldSimulatedBodies", &weld, 1, &ok);
            if (ok) { fn->call(copy, p); auto* r = fn->find_property(L"ReturnValue"); API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: attached -> %d", r ? (int)p[r->get_offset()] : -1); }
        }
    }
    {
        auto* fn = sc_fn(copy, L"K2_SetRelativeLocationAndRotation");
        if (fn != nullptr) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
            sm_put(fn, p, sizeof(p), L"NewLocation", loc, 24, &ok);
            sm_put(fn, p, sizeof(p), L"NewRotation", rot, 24, &ok);
            sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
            if (ok) fn->call(copy, p);
        }
        alignas(16) uint8_t q[64] = {0}; memcpy(q, scl, 24); copy->call_function(L"SetRelativeScale3D", q);
    }
    // The bones, and the one to move.
    s_sc_bone_ok = false;
    {
        int32_t nb = 0;
        if (auto* fn = sc_fn(copy, L"GetNumBones")) { alignas(16) uint8_t p[64] = {0}; fn->call(copy, p); auto* r = fn->find_property(L"ReturnValue"); if (r) nb = *reinterpret_cast<int32_t*>(p + r->get_offset()); }
        std::string want = trim_cfg(g_cfg.slide_copy_bone);
        for (auto& ch : want) ch = (char)tolower((unsigned char)ch);
        std::wstring names;
        auto* gb = sc_fn(copy, L"GetBoneName");
        for (int32_t i = 0; gb != nullptr && i < nb && i < 64; ++i) {
            alignas(16) uint8_t p[64] = {0}; bool ok = true;
            sm_put(gb, p, sizeof(p), L"BoneIndex", &i, sizeof(int32_t), &ok);
            auto* r = gb->find_property(L"ReturnValue");
            if (!ok || r == nullptr) break;
            gb->call(copy, p);
            API::FName bn; memcpy(&bn, p + r->get_offset(), sizeof(int32_t) * 2);
            std::wstring wn = bn.to_string();
            names += (i ? L", " : L"") + wn;
            std::string an(wn.begin(), wn.end());
            for (auto& ch : an) ch = (char)tolower((unsigned char)ch);
            if (!s_sc_bone_ok && !want.empty() && an.find(want) != std::string::npos) { s_sc_bone = bn; s_sc_bone_ok = true; }
        }
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: %d bones: %ls", nb, names.c_str());
        if (auto* gp = sc_fn(copy, L"GetParentBone")) {
            std::wstring tree;
            for (int32_t i = 0; gb != nullptr && i < nb && i < 64; ++i) {
                alignas(16) uint8_t p[64] = {0}; bool ok = true;
                sm_put(gb, p, sizeof(p), L"BoneIndex", &i, sizeof(int32_t), &ok);
                auto* r = gb->find_property(L"ReturnValue");
                if (!ok || r == nullptr) break;
                gb->call(copy, p);
                API::FName bn; memcpy(&bn, p + r->get_offset(), sizeof(int32_t) * 2);
                alignas(16) uint8_t q[64] = {0}; bool ok2 = true;
                sm_put(gp, q, sizeof(q), L"BoneName", &bn, sizeof(int32_t) * 2, &ok2);
                auto* r2 = gp->find_property(L"ReturnValue");
                if (!ok2 || r2 == nullptr) break;
                gp->call(copy, q);
                API::FName pn; memcpy(&pn, q + r2->get_offset(), sizeof(int32_t) * 2);
                tree += bn.to_string() + L"<-" + pn.to_string() + L"  ";
            }
            API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: hierarchy (bone<-parent): %ls", tree.c_str());
        }
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: slide bone '%s' -> %s", want.c_str(), s_sc_bone_ok ? "found" : "NOT FOUND (copy shows, nothing moves)");
    }
    sc_mirror_flags(src, copy);
    sc_dump_asset_users(mesh);
    s_sc_copy.set(copy); s_sc_src.set(src); s_sc_actor.set(actor);
    if (seq_mode) {
        // Single-node animation on the copy: the sequence's time is ours to set.
        s_sc_seq = TrackedObject{}; s_sc_len = 0.0f;
        auto* seq = find_anim_sequence(trim_cfg(g_cfg.slide_copy_seq));
        if (seq == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: no AnimSequence matching '%s'", trim_cfg(g_cfg.slide_copy_seq).c_str()); return true; }
        { alignas(16) uint8_t p[64] = {0}; p[0] = 1;   // EAnimationMode::AnimationSingleNode
          copy->call_function(L"SetAnimationMode", p); }
        { alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<API::UObject**>(p) = seq;
          copy->call_function(L"SetAnimation", p); }
        if (g_cfg.slide_copy_play == 0) { alignas(16) uint8_t p[64] = {0}; p[0] = 0; copy->call_function(L"SetPlayRate", p); }
        else if (g_cfg.slide_copy_play == 1) { alignas(16) uint8_t p[64] = {0}; p[0] = 1; copy->call_function(L"Play", p); }
        else { alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<float*>(p) = 0.001f; copy->call_function(L"SetPlayRate", p); }
        // Tick the pose whether or not the renderer has drawn the component yet: a Nanite-skinned
        // mesh with no evaluated pose may never get bone data, and so never draw (2026-09-04).
        { alignas(16) uint8_t p[64] = {0}; p[0] = 0;   // AlwaysTickPoseAndRefreshBones
          copy->call_function(L"SetVisibilityBasedAnimTickOption", p); }
        if (auto* pl = seq->get_property_data<float>(L"SequenceLength")) if (!IsBadReadPtr(pl, sizeof(float))) s_sc_len = *pl;
        s_sc_seq.set(seq);
        { alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<float*>(p) = s_sc_len; p[4] = 0; copy->call_function(L"SetPosition", p); }
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: single-node %ls, length %.3f s, parked at the end (gun at rest)", seq->get_full_name().c_str(), s_sc_len);
        return true;
    }
    if (g_cfg.slide_copy_follower && g_cfg.slide_copy_class == 0) {
        // The leader must refresh its bone transforms every tick even though it never renders.
        { alignas(16) uint8_t p[64] = {0}; p[0] = 0;   // AlwaysTickPoseAndRefreshBones
          copy->call_function(L"SetVisibilityBasedAnimTickOption", p); }
        sc_set_visibility(copy, false);
        auto* fcls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.SkeletalMeshComponent");
        auto* fol = fcls ? API::get()->add_component_by_class(actor, fcls, false) : nullptr;
        if (fol == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: follower spawn failed"); return true; }
        for (const wchar_t* fname : { L"SetSkinnedAssetAndUpdate", L"SetSkeletalMesh" }) {
            auto* fn = sc_fn(fol, fname);
            if (fn == nullptr) continue;
            alignas(16) uint8_t p[64] = {0}; bool ok = true; const bool reinit = true;
            sm_put(fn, p, sizeof(p), L"NewMesh", &mesh, sizeof(void*), &ok);
            sm_put(fn, p, sizeof(p), L"bReinitPose", &reinit, 1, &ok);
            if (ok) { fn->call(fol, p); break; }
        }
        { alignas(16) uint8_t p[64] = {0}; p[0] = 0; fol->call_function(L"SetCollisionEnabled", p); }
        if (parent != nullptr) {
            if (auto* fn = sc_fn(fol, L"K2_AttachToComponent")) {
                alignas(16) uint8_t p[128] = {0}; bool ok = true; const uint8_t rule = 0; const bool weld = false;
                sm_put(fn, p, sizeof(p), L"Parent", &parent, sizeof(void*), &ok);
                sm_put(fn, p, sizeof(p), L"SocketName", &socket, sizeof(int32_t) * 2, &ok);
                sm_put(fn, p, sizeof(p), L"LocationRule", &rule, 1, &ok);
                sm_put(fn, p, sizeof(p), L"RotationRule", &rule, 1, &ok);
                sm_put(fn, p, sizeof(p), L"ScaleRule", &rule, 1, &ok);
                sm_put(fn, p, sizeof(p), L"bWeldSimulatedBodies", &weld, 1, &ok);
                if (ok) fn->call(fol, p);
            }
        }
        if (auto* fn = sc_fn(fol, L"K2_SetRelativeLocationAndRotation")) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
            sm_put(fn, p, sizeof(p), L"NewLocation", loc, 24, &ok);
            sm_put(fn, p, sizeof(p), L"NewRotation", rot, 24, &ok);
            sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
            if (ok) fn->call(fol, p);
        }
        { alignas(16) uint8_t q[64] = {0}; memcpy(q, scl, 24); fol->call_function(L"SetRelativeScale3D", q); }
        // The follower takes its bone transforms from the leader. 5.1+ names it leader; older, master.
        bool led = false;
        for (const wchar_t* fname : { L"SetLeaderPoseComponent", L"SetMasterPoseComponent" }) {
            auto* fn = sc_fn(fol, fname);
            if (fn == nullptr) continue;
            alignas(16) uint8_t p[64] = {0}; bool ok = true; const bool force = true, tick = false;
            // First parameter = the leader, whatever its name in this engine version.
            for (API::FField* f = fn->get_child_properties(); f != nullptr; f = f->get_next()) {
                const auto* fnm = f->get_fname();
                if (fnm == nullptr) continue;
                const std::wstring nm = fnm->to_string();
                if (nm == L"ReturnValue") continue;
                if (f->get_class() && f->get_class()->get_name() == L"ObjectProperty") {
                    API::UObject* leader = (g_cfg.slide_copy_leader == 1) ? src : copy;
                    sm_put(fn, p, sizeof(p), nm.c_str(), &leader, sizeof(void*), &ok); break;
                }
            }
            sm_put(fn, p, sizeof(p), L"bForceUpdate", &force, 1, &ok);
            auto* tp = fn->find_property(L"bFollowerShouldTickPose"); if (tp == nullptr) tp = fn->find_property(L"bSlaveShouldTickPose");
            if (tp != nullptr) memcpy(p + tp->get_offset(), &tick, 1);
            if (ok) { fn->call(fol, p); led = true; API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: follower spawned, leader = %s, set via %ls", g_cfg.slide_copy_leader == 1 ? "the REAL mesh (draw test)" : "the poseable", fname); }
            break;
        }
        if (!led) API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: follower spawned but no leader-pose setter found");
        sc_set_visibility(fol, false); sc_set_visibility(fol, true);
        s_sc_follower.set(fol);
    }
    return true;
}
void sc_dump_asset_users(API::UObject* asset) {
    if (asset == nullptr) return;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count();
    int n = 0;
    for (int32_t i = 0; i < nn && n < 24; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring cls = class_name_of(o);
        if (cls.find(L"MeshComponent") == std::wstring::npos) continue;
        API::UObject* mesh = nullptr;
        for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh", L"StaticMesh" }) {
            auto** pm = o->get_property_data<API::UObject*>(nm);
            if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
        }
        if (mesh != asset) continue;
        ++n;
        auto rb = [&](const wchar_t* nm) -> int {
            auto* pr = o->get_class()->find_property(nm);
            if (pr == nullptr || pr->get_class() == nullptr || pr->get_class()->get_name() != L"BoolProperty") return -1;
            return static_cast<API::FBoolProperty*>(pr)->get_value_from_object(o) ? 1 : 0;
        };
        int fpt = -1;
        if (auto* pt = o->get_property_data<uint8_t>(L"FirstPersonPrimitiveType")) if (!IsBadReadPtr(pt, 1)) fpt = (int)*pt;
        API::UObject* par = nullptr;
        if (auto** pp = o->get_property_data<API::UObject*>(L"AttachParent")) if (!IsBadReadPtr(pp, sizeof(void*))) par = *pp;
        double scl[3] = {0, 0, 0};
        if (auto* ps = o->get_property_data<double>(L"RelativeScale3D")) if (!IsBadReadPtr(ps, 24)) { scl[0] = ps[0]; scl[1] = ps[1]; scl[2] = ps[2]; }
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY asset user #%d: %ls '%ls' in %ls | mainpass=%d depth=%d visible=%d hidden=%d ownerNoSee=%d onlyOwner=%d fpType=%d | parent %ls '%ls' | scale (%.3f %.3f %.3f)",
                             n, cls.c_str(), o->get_fname() ? o->get_fname()->to_string().c_str() : L"?",
                             o->get_full_name().c_str(),
                             rb(L"bRenderInMainPass"), rb(L"bRenderInDepthPass"), rb(L"bVisible"), rb(L"bHiddenInGame"),
                             rb(L"bOwnerNoSee"), rb(L"bOnlyOwnerSee"), fpt,
                             par ? class_name_of(par).c_str() : L"null",
                             (par && par->get_fname()) ? par->get_fname()->to_string().c_str() : L"?",
                             scl[0], scl[1], scl[2]);
    }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: %d component(s) use this asset", n);
}
// Double-precision transform helpers for the alignment (UE5 FTransform is doubles).
struct QD { double x, y, z, w; };
QD qd_mul(const QD& a, const QD& b) {
    return QD{ a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
               a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
               a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
               a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z };
}
QD qd_conj(const QD& q) { return QD{-q.x, -q.y, -q.z, q.w}; }
void qd_rot(const QD& q, const double v[3], double out[3]) {
    const double cx = q.y*v[2] - q.z*v[1], cy = q.z*v[0] - q.x*v[2], cz = q.x*v[1] - q.y*v[0];
    const double dx = q.y*cz - q.z*cy,     dy = q.z*cx - q.x*cz,     dz = q.x*cy - q.y*cx;
    out[0] = v[0] + 2.0*(q.w*cx + dx); out[1] = v[1] + 2.0*(q.w*cy + dy); out[2] = v[2] + 2.0*(q.w*cz + dz);
}
bool sc_socket_world(API::UObject* comp, const API::FName& name, XformD* out) {
    auto* fn = sc_fn(comp, L"GetSocketTransform");
    if (fn == nullptr) return false;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const uint8_t space = 0;   // RTS_World
    sm_put(fn, p, sizeof(p), L"InSocketName", &name, sizeof(int32_t) * 2, &ok);
    sm_put(fn, p, sizeof(p), L"TransformSpace", &space, 1, &ok);
    auto* r = fn->find_property(L"ReturnValue");
    if (!ok || r == nullptr) return false;
    fn->call(comp, p);
    memcpy(out, p + r->get_offset(), sizeof(XformD));
    return true;
}
bool sc_component_world(API::UObject* comp, XformD* out) {
    auto* fn = sc_fn(comp, L"K2_GetComponentToWorld");
    if (fn == nullptr) return false;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    auto* r = fn->find_property(L"ReturnValue");
    if (r == nullptr) return false;
    fn->call(comp, p);
    memcpy(out, p + r->get_offset(), sizeof(XformD));
    return true;
}
// Move the copy's component so its root bone lands on the real mesh's root bone:
// C' = R_real * inverse(R_copy) * C  (rotation and translation; scale left alone).
void sc_align_roots(API::UObject* src, API::UObject* copy) {
    static API::FName s_root; static std::string s_root_name; static bool s_have = false;
    const std::string want = trim_cfg(g_cfg.slide_copy_root);
    if (!s_have || want != s_root_name) { std::wstring w(want.begin(), want.end()); s_root = make_fname(w.c_str()); s_root_name = want; s_have = true; }
    XformD rr{}, rc{}, c{};
    if (!sc_socket_world(src, s_root, &rr) || !sc_socket_world(copy, s_root, &rc) || !sc_component_world(copy, &c)) return;
    const QD qr{rr.qx, rr.qy, rr.qz, rr.qw}, qc{rc.qx, rc.qy, rc.qz, rc.qw}, qC{c.qx, c.qy, c.qz, c.qw};
    // D = R_real * inverse(R_copy): rotation qd = qr * conj(qc); translation td = tr - qd * tc
    const QD qd = qd_mul(qr, qd_conj(qc));
    const double tc[3] = {rc.tx, rc.ty, rc.tz}; double tcr[3]; qd_rot(qd, tc, tcr);
    const double td[3] = {rr.tx - tcr[0], rr.ty - tcr[1], rr.tz - tcr[2]};
    // C' = D * C: rotation qd*qC, translation td + qd * tC
    const QD qn = qd_mul(qd, qC);
    const double tC[3] = {c.tx, c.ty, c.tz}; double tCr[3]; qd_rot(qd, tC, tCr);
    XformD n = c;
    n.qx = qn.x; n.qy = qn.y; n.qz = qn.z; n.qw = qn.w;
    n.tx = td[0] + tCr[0]; n.ty = td[1] + tCr[1]; n.tz = td[2] + tCr[2];
    if (auto* fn = sc_fn(copy, L"K2_SetWorldTransform")) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
        sm_put(fn, p, sizeof(p), L"NewTransform", &n, sizeof(XformD), &ok);
        sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
        sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
        if (ok) fn->call(copy, p);
    }
    if (g_cfg.slide_log && (s_sc_ticks % 90u) == 1u)
        API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY align: real root (%.1f %.1f %.1f) copy root (%.1f %.1f %.1f) copy comp (%.1f %.1f %.1f) -> comp' (%.1f %.1f %.1f)",
                             rr.tx, rr.ty, rr.tz, rc.tx, rc.ty, rc.tz, c.tx, c.ty, c.tz, n.tx, n.ty, n.tz);
}
void slide_copy_tick() {
    auto* src = reload_weapon_default_comp();
    auto* actor = fp_weapon_actor();
    const bool usable = g_cfg.slide_copy && reload_rack_available() && src != nullptr && actor != nullptr && slide_weapon_ok();
    const std::string key = weapon_key();
    if (!usable || (s_sc_copy.get() != nullptr && (key != s_sc_key || s_sc_src.get() != src))) {
        sc_teardown(usable ? "weapon changed" : "unavailable");
        if (!usable) return;
    }
    if (s_sc_copy.get() == nullptr) {
        if (now_ticks() - s_sc_retry_at < ms_to_ticks(1500)) return;
        s_sc_retry_at = now_ticks();
        if (!sc_spawn(actor, src)) return;
        s_sc_key = key;
    }
    auto* copy = s_sc_copy.get();
    if (copy == nullptr) return;
    ++s_sc_ticks;
    if (g_cfg.slide_copy_mode == 1) {
        if (s_sc_seq.get() != nullptr && s_sc_len > 0.0f) {
            const float pull = g_slide_pull.load(std::memory_order_relaxed);
            float frac = (g_cfg.slide_travel > 0.0f) ? std::fmin(1.0f, std::fmax(0.0f, pull / g_cfg.slide_travel)) : 0.0f;
            if (s_true_empty || s_sl_lock_pending || s_sl_locked_back) frac = 1.0f;   // locked back
            float t = s_sc_len - frac * g_cfg.slide_copy_rel;
            if (g_cfg.slide_copy_sweep) {
                static long long s_t0 = 0;
                if (s_t0 == 0) s_t0 = now_ticks();
                const double sec = (double)(now_ticks() - s_t0) / (double)ms_to_ticks(1000);
                t = s_sc_len - 0.6f + (float)std::fmod(sec * 0.1, 0.6);   // 0.1 s of animation per second
                if (g_cfg.slide_log) { static long long s_said = 0; if (now_ticks() - s_said > ms_to_ticks(500)) { s_said = now_ticks(); API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY sweep t=%.3f (end-%.3f)", t, s_sc_len - t); } }
            }
            if (t < 0.0f) t = 0.0f;
            if (g_cfg.slide_copy_play != 1) {
                alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<float*>(p) = t; p[4] = 0;
                copy->call_function(L"SetPosition", p);
            }
        }
        if (g_cfg.slide_copy_align) sc_align_roots(src, copy);
        if (g_cfg.slide_log && (s_sc_ticks % 90u) == 2u) {
            int rr = -1;
            if (auto* pr = copy->get_class()->find_property(L"bRecentlyRendered")) if (pr->get_class() && pr->get_class()->get_name() == L"BoolProperty") rr = static_cast<API::FBoolProperty*>(pr)->get_value_from_object(copy) ? 1 : 0;
            API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: copy bRecentlyRendered=%d", rr);
            // Where the animated bones land relative to the root: a degenerate or far pose is a
            // number here, not a theory.
            XformD r0{}, rb{}, rs{}, rr0{}, rrb{};
            const API::FName nroot = make_fname(L"Root_M"), nbody = make_fname(L"Body_M"), nslide = make_fname(L"Slide_M");
            if (sc_socket_world(copy, nroot, &r0) && sc_socket_world(copy, nbody, &rb) && sc_socket_world(copy, nslide, &rs)
                && sc_socket_world(src, nroot, &rr0) && sc_socket_world(src, nbody, &rrb)) {
                API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY bones: copy body-root (%.1f %.1f %.1f) slide-root (%.1f %.1f %.1f) scale %.2f | real body-root (%.1f %.1f %.1f) scale %.2f",
                                     rb.tx - r0.tx, rb.ty - r0.ty, rb.tz - r0.tz, rs.tx - r0.tx, rs.ty - r0.ty, rs.tz - r0.tz, rb.sx,
                                     rrb.tx - rr0.tx, rrb.ty - rr0.ty, rrb.tz - rr0.tz, rrb.sx);
            }
        }
        sc_hide_real(src, true);
        s_sc_hidden = true;
        if (s_sc_ticks == 1 && g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: live (single-node); real mesh hidden");
        return;
    }
    // The real mesh's pose, then our slide, then the real mesh out of sight.
    if (auto* fn = sc_fn(copy, L"CopyPoseFromSkeletalComponent")) {
        alignas(16) uint8_t p[64] = {0}; bool ok = true;
        sm_put(fn, p, sizeof(p), L"InComponentToCopy", &src, sizeof(void*), &ok);
        if (ok) fn->call(copy, p);
    }
    const float pull_cm = g_slide_pull.load(std::memory_order_relaxed) * 304.8f * g_cfg.slide_copy_sign;
    const float off_cm = pull_cm + g_cfg.slide_copy_test;
    if (s_sc_bone_ok && std::fabs(off_cm) > 0.0005f) {
        auto* gt = sc_fn(copy, L"GetBoneTransformByName");
        auto* st = sc_fn(copy, L"SetBoneTransformByName");
        if (gt != nullptr && st != nullptr) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const uint8_t space = 1;   // ComponentSpace
            sm_put(gt, p, sizeof(p), L"BoneName", &s_sc_bone, sizeof(int32_t) * 2, &ok);
            sm_put(gt, p, sizeof(p), L"BoneSpace", &space, 1, &ok);
            auto* r = gt->find_property(L"ReturnValue");
            if (ok && r != nullptr) {
                gt->call(copy, p);
                XformD t; memcpy(&t, p + r->get_offset(), sizeof(XformD));
                // The bone-local axis in component space: rotate the unit axis by the bone's rotation.
                double ax[3] = {0, 0, 0}; ax[g_cfg.slide_copy_axis] = 1.0;
                const double qx = t.qx, qy = t.qy, qz = t.qz, qw = t.qw;
                // v' = v + 2*w*(q x v) + 2*(q x (q x v))
                double cx = qy * ax[2] - qz * ax[1], cy = qz * ax[0] - qx * ax[2], cz = qx * ax[1] - qy * ax[0];
                double dx = qy * cz - qz * cy,       dy = qz * cx - qx * cz,       dz = qx * cy - qy * cx;
                const double vx = ax[0] + 2.0 * (qw * cx + dx), vy = ax[1] + 2.0 * (qw * cy + dy), vz = ax[2] + 2.0 * (qw * cz + dz);
                t.tx -= vx * off_cm; t.ty -= vy * off_cm; t.tz -= vz * off_cm;
                alignas(16) uint8_t q[RIG_PARAM_BUF] = {0}; bool ok2 = true;
                sm_put(st, q, sizeof(q), L"BoneName", &s_sc_bone, sizeof(int32_t) * 2, &ok2);
                sm_put(st, q, sizeof(q), L"InTransform", &t, sizeof(XformD), &ok2);
                sm_put(st, q, sizeof(q), L"BoneSpace", &space, 1, &ok2);
                if (ok2) st->call(copy, q);
                if (g_cfg.slide_log && (s_sc_ticks % 60u) == 0u)
                    API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: bone at (%.2f %.2f %.2f) axis%d=(%.2f %.2f %.2f) offset %.2f cm",
                                         t.tx, t.ty, t.tz, g_cfg.slide_copy_axis, vx, vy, vz, off_cm);
            }
        }
    }
    sc_hide_real(src, true);
    s_sc_hidden = true;
    if (s_sc_ticks == 1 && g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDECOPY: live; real mesh hidden");
}

// ---- SLIDEPART v2. See Config.hpp. The gun as the game's own static parts, each riding its bone.
struct SlidePart { TrackedObject comp; API::FName bone; std::wstring bone_name; XformD bind; bool have_bind; bool is_slide; bool is_mag; bool bone_pivot; bool claimed; API::UObject* mesh; bool dropped; };
TrackedObject s_sp_dropped, s_sp_dropped_proxy; long long s_sp_dropped_at = 0;
// The real mesh's own child components, moved onto our body part while the rebuild lives.
struct SpKid { TrackedObject comp; API::FName socket; double loc[3]; double rot[3]; double scl[3]; };
SpKid s_sp_kids[16]; int s_sp_nkids = 0;
void sp_attach(API::UObject* c, API::UObject* parent, const API::FName& socket, uint8_t rule) {
    auto* fn = sc_fn(c, L"K2_AttachToComponent"); if (fn == nullptr) return;
    alignas(16) uint8_t p[128] = {0}; bool ok = true; const bool weld = false;
    sm_put(fn, p, sizeof(p), L"Parent", &parent, sizeof(void*), &ok);
    sm_put(fn, p, sizeof(p), L"SocketName", &socket, sizeof(int32_t) * 2, &ok);
    sm_put(fn, p, sizeof(p), L"LocationRule", &rule, 1, &ok);
    sm_put(fn, p, sizeof(p), L"RotationRule", &rule, 1, &ok);
    sm_put(fn, p, sizeof(p), L"ScaleRule", &rule, 1, &ok);
    sm_put(fn, p, sizeof(p), L"bWeldSimulatedBodies", &weld, 1, &ok);
    if (ok) fn->call(c, p);
}
void sp_kids_restore(API::UObject* src) {
    for (int i = 0; i < s_sp_nkids; ++i) {
        auto* c = s_sp_kids[i].comp.get();
        if (c == nullptr || src == nullptr) continue;
        sp_attach(c, src, s_sp_kids[i].socket, 0);   // KeepRelative, then the relative transform back
        if (auto* fn = sc_fn(c, L"K2_SetRelativeLocationAndRotation")) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
            sm_put(fn, p, sizeof(p), L"NewLocation", s_sp_kids[i].loc, 24, &ok);
            sm_put(fn, p, sizeof(p), L"NewRotation", s_sp_kids[i].rot, 24, &ok);
            sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
            if (ok) fn->call(c, p);
        }
        { alignas(16) uint8_t q[64] = {0}; memcpy(q, s_sp_kids[i].scl, 24); c->call_function(L"SetRelativeScale3D", q); }
        { alignas(16) uint8_t q[64] = {0}; c->call_function(L"SetAbsolute", q); }   // all three false again
        s_sp_kids[i] = SpKid{};
    }
    s_sp_nkids = 0;
}
// Every component attached to the real mesh: logged, and moved per slide_part_kids.
void sp_kids_take(API::UObject* src, API::UObject* body_part) {
    s_sp_nkids = 0;
    weapon_components([&](API::UObject* c) {
        if (s_sp_nkids >= 16) return false;
        if (c == src) return true;
        auto** pp = c->get_property_data<API::UObject*>(L"AttachParent");
        if (pp == nullptr || IsBadReadPtr(pp, sizeof(void*)) || *pp != src) return true;
        SpKid& k = s_sp_kids[s_sp_nkids];
        API::UObject* par = nullptr;
        if (!sc_read_attach(c, &par, &k.socket, k.loc, k.rot, k.scl)) return true;
        const auto* fn = c->get_fname();
        std::wstring mesh = L"-";
        if (auto** pm = c->get_property_data<API::UObject*>(L"StaticMesh")) if (!IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr && (*pm)->get_fname()) mesh = (*pm)->get_fname()->to_string();
        API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART kid: %ls '%ls' mesh %ls at socket '%ls' rel (%.1f %.1f %.1f) -> %s",
                             class_name_of(c).c_str(), fn ? fn->to_string().c_str() : L"?", mesh.c_str(), k.socket.to_string().c_str(),
                             k.loc[0], k.loc[1], k.loc[2],
                             g_cfg.slide_part_kids == 1 ? "onto our body part" : (g_cfg.slide_part_kids == 2 ? "absolute scale" : "left"));
        if (g_cfg.slide_part_kids == 1 && body_part != nullptr) {
            k.comp.set(c);
            sp_attach(c, body_part, make_fname(L"None"), 1);   // KeepWorld
            ++s_sp_nkids;
        } else if (g_cfg.slide_part_kids == 2) {
            k.comp.set(c);
            { alignas(16) uint8_t q[64] = {0}; q[2] = 1; c->call_function(L"SetAbsolute", q); }   // scale absolute
            { alignas(16) uint8_t q[64] = {0}; auto* d = reinterpret_cast<double*>(q); d[0] = k.scl[0]; d[1] = k.scl[1]; d[2] = k.scl[2]; c->call_function(L"SetRelativeScale3D", q); }
            ++s_sp_nkids;
        }
        return true;
    });
}
SlidePart     s_sp_parts[12];
int           s_sp_count = 0;
TrackedObject s_sp_src;
std::string   s_sp_key;
bool          s_sp_hidden = false;
long long     s_sp_retry_at = 0;
uint32_t      s_sp_ticks = 0;
bool sp_socket_component(API::UObject* comp, const API::FName& name, XformD* out) {
    auto* fn = sc_fn(comp, L"GetSocketTransform");
    if (fn == nullptr) return false;
    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const uint8_t space = 2;   // RTS_Component
    sm_put(fn, p, sizeof(p), L"InSocketName", &name, sizeof(int32_t) * 2, &ok);
    sm_put(fn, p, sizeof(p), L"TransformSpace", &space, 1, &ok);
    auto* r = fn->find_property(L"ReturnValue");
    if (!ok || r == nullptr) return false;
    fn->call(comp, p);
    memcpy(out, p + r->get_offset(), sizeof(XformD));
    return true;
}
// THE PART'S GEOMETRIC CENTRE IN THE WORLD (2026-09-06): the pump's socket and component sit at
// the gun's root (the mesh pivot is at the receiver), so only the geometry's centre puts the zone
// on the pump. Three sources, first non-zero wins: K2_GetComponentBounds' Origin; the component's
// GetLocalBounds (Min, Max) through K2_GetComponentToWorld; the static mesh asset's
// GetBoundingBox through the same transform.
bool part_world_centre(API::UObject* c, Vec3* out, const char** how) {
    if (c == nullptr || out == nullptr) return false;
    auto nonzero = [](const double* v) { return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]) && (std::fabs(v[0]) + std::fabs(v[1]) + std::fabs(v[2]) > 1e-3); };
    {   // 1. K2_GetComponentBounds(Origin, BoxExtent, SphereRadius): Origin at 0
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        c->call_function(L"K2_GetComponentBounds", p);
        const double* o = reinterpret_cast<const double*>(p);
        if (nonzero(o)) { *out = Vec3{(float)o[0], (float)o[1], (float)o[2]}; *how = "bounds"; return true; }
    }
    // The component's world transform: FTransform of doubles, quat @0, translation @0x20, scale @0x40.
    alignas(16) uint8_t tp[RIG_PARAM_BUF] = {0};
    c->call_function(L"K2_GetComponentToWorld", tp);
    const double* q = reinterpret_cast<const double*>(tp);
    const double* tr = reinterpret_cast<const double*>(tp + 0x20);
    const double* sc = reinterpret_cast<const double*>(tp + 0x40);
    const bool have_xf = std::isfinite(q[3]) && std::fabs(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] - 1.0) < 0.05;
    auto to_world = [&](const double local[3], Vec3* w) {
        const double sl[3] = {local[0] * sc[0], local[1] * sc[1], local[2] * sc[2]};
        double r[3]; qd_rot(QD{q[0], q[1], q[2], q[3]}, sl, r);
        *w = Vec3{(float)(r[0] + tr[0]), (float)(r[1] + tr[1]), (float)(r[2] + tr[2])};
    };
    if (have_xf) {
        {   // 2. GetLocalBounds(Min @0, Max @0x18)
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
            c->call_function(L"GetLocalBounds", p);
            const double* mn = reinterpret_cast<const double*>(p); const double* mx = reinterpret_cast<const double*>(p + 0x18);
            if (nonzero(mn) || nonzero(mx)) { const double ctr[3] = {(mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5}; to_world(ctr, out); *how = "localbounds"; return true; }
        }
        {   // 3. the asset: StaticMesh->GetBoundingBox() = FBox {Min @0, Max @0x18, IsValid}
            auto** pm = c->get_property_data<API::UObject*>(L"StaticMesh");
            if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr && !IsBadReadPtr(*pm, sizeof(void*))) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                (*pm)->call_function(L"GetBoundingBox", p);
                const double* mn = reinterpret_cast<const double*>(p); const double* mx = reinterpret_cast<const double*>(p + 0x18);
                if (nonzero(mn) || nonzero(mx)) { const double ctr[3] = {(mn[0] + mx[0]) * 0.5, (mn[1] + mx[1]) * 0.5, (mn[2] + mx[2]) * 0.5}; to_world(ctr, out); *how = "assetbox"; return true; }
            }
        }
    }
    *how = "none";
    return false;
}
API::FName sp_slide_bone_name() {
    static API::FName s_name; static std::string s_last; static bool s_have = false;
    const std::string want = trim_cfg(g_cfg.slide_part_bone);
    if (!s_have || want != s_last) { std::wstring w(want.begin(), want.end()); s_name = make_fname(w.c_str()); s_last = want; s_have = true; }
    return s_name;
}
// The arms hide's recipe: keep the pose evaluating, then hide the bone (PBO_None). Re-issued
// every tick here, and the engine is asked whether it holds the bone hidden.
void sp_hide_bone(API::UObject* src, bool hide) {
    if (src == nullptr) return;
    const API::FName bone = sp_slide_bone_name();
    if (hide) { alignas(16) uint8_t p[64] = {0}; p[0] = 0; src->call_function(L"SetVisibilityBasedAnimTickOption", p); }
    auto* fn = sc_fn(src, hide ? L"HideBoneByName" : L"UnHideBoneByName");
    if (fn == nullptr) return;
    alignas(16) uint8_t p[64] = {0}; bool ok = true; const uint8_t pbo = (uint8_t)g_cfg.slide_hide_pbo;
    sm_put(fn, p, sizeof(p), L"BoneName", &bone, sizeof(int32_t) * 2, &ok);
    if (hide) { auto* pr = fn->find_property(L"PhysBodyOption"); if (pr) memcpy(p + pr->get_offset(), &pbo, 1); }
    if (ok) fn->call(src, p);
}
int sp_is_bone_hidden(API::UObject* src) {
    auto* fn = sc_fn(src, L"IsBoneHiddenByName");
    if (fn == nullptr) return -1;
    const API::FName bone = sp_slide_bone_name();
    alignas(16) uint8_t p[64] = {0}; bool ok = true;
    sm_put(fn, p, sizeof(p), L"BoneName", &bone, sizeof(int32_t) * 2, &ok);
    auto* r = fn->find_property(L"ReturnValue");
    if (!ok || r == nullptr) return -1;
    fn->call(src, p);
    return p[r->get_offset()] ? 1 : 0;
}
// The real mesh's material slots, by name, once: which one is the slide?
void sp_log_materials(API::UObject* src) {
    int32_t n = 0;
    if (auto* fn = sc_fn(src, L"GetNumMaterials")) { alignas(16) uint8_t p[64] = {0}; fn->call(src, p); auto* r = fn->find_property(L"ReturnValue"); if (r) n = *reinterpret_cast<int32_t*>(p + r->get_offset()); }
    std::wstring line;
    auto* gm = sc_fn(src, L"GetMaterial");
    for (int32_t i = 0; gm != nullptr && i < n && i < 32; ++i) {
        alignas(16) uint8_t p[64] = {0}; bool ok = true;
        sm_put(gm, p, sizeof(p), L"ElementIndex", &i, sizeof(int32_t), &ok);
        auto* r = gm->find_property(L"ReturnValue");
        if (!ok || r == nullptr) break;
        gm->call(src, p);
        auto* m = *reinterpret_cast<API::UObject**>(p + r->get_offset());
        line += std::to_wstring(i) + L":" + ((m && !IsBadReadPtr(m, sizeof(void*)) && m->get_fname()) ? m->get_fname()->to_string() : L"null") + L"  ";
    }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: real mesh has %d material slot(s): %ls", n, line.c_str());
}
// ShowMaterialSection(int32 MaterialID, int32 SectionIndex, bool bShow, int32 LODIndex)
void sp_show_section(API::UObject* src, int32_t material_id, bool show) {
    auto* fn = sc_fn(src, L"ShowMaterialSection");
    if (fn == nullptr) { static bool s_said = false; if (!s_said) { s_said = true; API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: no ShowMaterialSection"); } return; }
    for (int32_t lod = 0; lod < 4; ++lod) {
        alignas(16) uint8_t p[64] = {0}; bool ok = true; const int32_t sec = 0; const bool b = show;
        sm_put(fn, p, sizeof(p), L"MaterialID", &material_id, sizeof(int32_t), &ok);
        sm_put(fn, p, sizeof(p), L"SectionIndex", &sec, sizeof(int32_t), &ok);
        sm_put(fn, p, sizeof(p), L"bShow", &b, 1, &ok);
        sm_put(fn, p, sizeof(p), L"LODIndex", &lod, sizeof(int32_t), &ok);
        if (ok) fn->call(src, p);
    }
}
int s_sp_section_hidden = -1;
int s_sp_mode = -1;
// The real component's bone transform arrays, found by SHAPE: a TArray header {data, num, max}
// with num == bone count whose elements are UE5 double FTransforms (unit quaternion, sane scale).
// The component-space arrays carry the slide at its component position (5.5, 0, 13 on the
// magnum); the bone-space array carries it relative to its parent. Logged, never assumed.
struct BoneArray { uintptr_t data; int32_t off; bool component_space; };
BoneArray s_sp_arrays[6]; int s_sp_narrays = 0; int s_sp_slide_index = -1;
void sp_find_bone_arrays(API::UObject* src, int32_t nbones, int32_t slide_index, const XformD& slide_component) {
    s_sp_narrays = 0;
    const uint8_t* base = reinterpret_cast<const uint8_t*>(src);
    constexpr int32_t SPAN = 0x1800;
    if (IsBadReadPtr(base, SPAN)) return;
    for (int32_t off = 0; off + 16 <= SPAN && s_sp_narrays < 6; off += 8) {
        const uintptr_t data = *reinterpret_cast<const uintptr_t*>(base + off);
        const int32_t num = *reinterpret_cast<const int32_t*>(base + off + 8);
        const int32_t mx  = *reinterpret_cast<const int32_t*>(base + off + 12);
        if (num != nbones || mx < num || mx > num + 64 || data == 0 || (data & 0xF) != 0) continue;
        if (IsBadReadPtr((const void*)data, sizeof(XformD) * (size_t)num)) continue;
        bool ok = true;
        for (int32_t i = 0; i < num && ok; ++i) {
            const XformD* t = reinterpret_cast<const XformD*>(data + sizeof(XformD) * (size_t)i);
            const double qn = t->qx*t->qx + t->qy*t->qy + t->qz*t->qz + t->qw*t->qw;
            if (!(qn > 0.9 && qn < 1.1)) ok = false;
            if (!(std::fabs(t->sx) < 100.0 && std::fabs(t->sy) < 100.0 && std::fabs(t->sz) < 100.0)) ok = false;
            if (!(std::fabs(t->tx) < 100000.0 && std::fabs(t->ty) < 100000.0 && std::fabs(t->tz) < 100000.0)) ok = false;
        }
        if (!ok) continue;
        const XformD* sl = reinterpret_cast<const XformD*>(data + sizeof(XformD) * (size_t)slide_index);
        const bool comp = std::fabs(sl->tx - slide_component.tx) < 0.05 && std::fabs(sl->ty - slide_component.ty) < 0.05 && std::fabs(sl->tz - slide_component.tz) < 0.05;
        s_sp_arrays[s_sp_narrays++] = BoneArray{data, off, comp};
        API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART far: transform array at +0x%X (%d bones): slide entry t=(%.2f %.2f %.2f) s=(%.2f %.2f %.2f) -> %s",
                             (unsigned)off, num, sl->tx, sl->ty, sl->tz, sl->sx, sl->sy, sl->sz, comp ? "COMPONENT space" : "bone space (parent-relative)");
    }
    if (s_sp_narrays == 0) API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART far: no transform array of %d bones found in the component", nbones);
}
void sp_park_far_tick(API::UObject* src) {
    if (s_sp_slide_index < 0) return;
    int wrote = 0;
    for (int i = 0; i < s_sp_narrays; ++i) {
        const bool want_comp = (g_cfg.slide_far_array == 1);
        if (s_sp_arrays[i].component_space != want_comp) continue;
        auto* t = reinterpret_cast<XformD*>(s_sp_arrays[i].data + sizeof(XformD) * (size_t)s_sp_slide_index);
        if (IsBadWritePtr(t, sizeof(XformD))) continue;
        t->tz = (double)g_cfg.slide_far_cm;
        ++wrote;
    }
    if (g_cfg.slide_log && (s_sp_ticks % 90u) == 7u) {
        XformD now{};
        const bool have = sp_socket_component(src, sp_slide_bone_name(), &now);
        API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART far: wrote %d array(s); the slide bone now reads component z=%.1f (%s)",
                             wrote, have ? now.tz : 0.0, (have && now.tz < -100.0) ? "PARKED" : "still in place");
    }
}
// ---- approach C: morph targets on the real mesh
void sp_log_morphs(API::UObject* src) {
    API::UObject* mesh = nullptr;
    for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
        auto** pm = src->get_property_data<API::UObject*>(nm);
        if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
    }
    if (mesh == nullptr) return;
    auto* pr = mesh->get_class()->find_property(L"MorphTargets");
    if (pr == nullptr) { API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART morph: no MorphTargets property on %ls", class_name_of(mesh).c_str()); return; }
    const uint8_t* arr = reinterpret_cast<const uint8_t*>(mesh) + pr->get_offset();
    if (IsBadReadPtr(arr, 16)) return;
    const uintptr_t data = *reinterpret_cast<const uintptr_t*>(arr);
    const int32_t num = *reinterpret_cast<const int32_t*>(arr + 8);
    std::wstring names;
    for (int32_t i = 0; i < num && i < 32 && data != 0; ++i) {
        auto* o = *reinterpret_cast<API::UObject* const*>(data + sizeof(void*) * (size_t)i);
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*)) || o->get_fname() == nullptr) continue;
        names += o->get_fname()->to_string() + L"  ";
    }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART morph: %d morph target(s): %ls", num, names.empty() ? L"(none)" : names.c_str());
}
void sp_morph_tick(API::UObject* src) {
    const std::string spec = trim_cfg(g_cfg.slide_morph);
    const size_t comma = spec.find(',');
    if (spec.empty() || comma == std::string::npos) return;
    const std::string name = spec.substr(0, comma);
    const float w = (float)atof(spec.c_str() + comma + 1);
    auto* fn = sc_fn(src, L"SetMorphTarget");
    if (fn == nullptr) return;
    std::wstring wn(name.begin(), name.end());
    API::FName mn = make_fname(wn.c_str());
    alignas(16) uint8_t p[64] = {0}; bool ok = true; const bool rm = false;
    sm_put(fn, p, sizeof(p), L"MorphTargetName", &mn, sizeof(int32_t) * 2, &ok);
    sm_put(fn, p, sizeof(p), L"Value", &w, sizeof(float), &ok);
    auto* pz = fn->find_property(L"bRemoveZeroWeight"); if (pz) memcpy(p + pz->get_offset(), &rm, 1);
    if (ok) fn->call(src, p);
}
TrackedObject s_sp_mat_orig; int s_sp_mat_slot = -1;
API::UObject* sp_get_material(API::UObject* src, int32_t slot) {
    auto* gm = sc_fn(src, L"GetMaterial"); if (gm == nullptr) return nullptr;
    alignas(16) uint8_t p[64] = {0}; bool ok = true;
    sm_put(gm, p, sizeof(p), L"ElementIndex", &slot, sizeof(int32_t), &ok);
    auto* r = gm->find_property(L"ReturnValue"); if (!ok || r == nullptr) return nullptr;
    gm->call(src, p);
    return *reinterpret_cast<API::UObject**>(p + r->get_offset());
}
void sp_set_material(API::UObject* src, int32_t slot, API::UObject* mat) {
    auto* fn = sc_fn(src, L"SetMaterial"); if (fn == nullptr) return;
    alignas(16) uint8_t p[64] = {0}; bool ok = true;
    sm_put(fn, p, sizeof(p), L"ElementIndex", &slot, sizeof(int32_t), &ok);
    sm_put(fn, p, sizeof(p), L"Material", &mat, sizeof(void*), &ok);
    if (ok) fn->call(src, p);
}
// A loaded material whose name carries the substring; candidates that sound invisible are logged once.
API::UObject* sp_find_material(const std::string& sub) {
    std::wstring want(sub.begin(), sub.end()); for (auto& ch : want) ch = (wchar_t)towlower(ch);
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return nullptr;
    const int32_t nn = arr->get_object_count();
    API::UObject* hit = nullptr;
    static bool s_listed = false; int listed = 0; std::wstring cands;
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        const std::wstring cls = class_name_of(o);
        if (cls != L"Material" && cls != L"MaterialInstanceConstant" && cls != L"MaterialInstanceDynamic") continue;
        const auto* fn = o->get_fname(); if (fn == nullptr) continue;
        std::wstring nm = fn->to_string(); std::wstring lo = nm; for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        if (hit == nullptr && !want.empty() && lo.find(want) != std::wstring::npos) hit = o;
        if (!s_listed && listed < 24 && (lo.find(L"invis") != std::wstring::npos || lo.find(L"transp") != std::wstring::npos || lo.find(L"hidden") != std::wstring::npos ||
                                        lo.find(L"occlu") != std::wstring::npos || lo.find(L"empty") != std::wstring::npos || lo.find(L"clear") != std::wstring::npos || lo.find(L"nodraw") != std::wstring::npos)) {
            cands += nm + L"  "; ++listed;
        }
    }
    if (!s_listed) { s_listed = true; API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: invisible-sounding materials loaded: %ls", cands.empty() ? L"(none)" : cands.c_str()); }
    return hit;
}
void sp_teardown(const char* why) {
    if (s_sp_mat_slot >= 0) { if (auto* src = s_sp_src.get()) sp_set_material(src, s_sp_mat_slot, s_sp_mat_orig.get()); s_sp_mat_slot = -1; s_sp_mat_orig = TrackedObject{}; }
    if (s_sp_section_hidden >= 0) { if (auto* src = s_sp_src.get()) sp_show_section(src, s_sp_section_hidden, true); s_sp_section_hidden = -1; }
    for (int i = 0; i < s_sp_count; ++i) {
        if (auto* c = s_sp_parts[i].comp.get()) { alignas(16) uint8_t p[64] = {0}; ue_destroy_component(c); }
        s_sp_parts[i] = SlidePart{};
    }
    if (s_sp_hidden) { if (auto* src = s_sp_src.get()) { sc_set_scale(src, 1.0); sc_set_visibility(src, true); sp_hide_bone(src, false); sp_kids_restore(src); } }
    if (s_sp_count > 0 || s_sp_hidden) { if (g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: torn down (%s)", why); }
    g_sl_part_valid.store(false, std::memory_order_relaxed);
    if (auto* d = s_sp_dropped.get()) ue_destroy_component(d);
    if (auto* px = s_sp_dropped_proxy.get()) ue_destroy_component(px);
    s_sp_dropped = TrackedObject{}; s_sp_dropped_proxy = TrackedObject{};
    s_sp_count = 0; s_sp_src = TrackedObject{}; s_sp_hidden = false; s_sp_key.clear(); s_sp_ticks = 0;
    g_slide_rack_found = false;   // the native mode re-sets it when its part is found
}
API::UObject* sp_find_static_mesh(const std::wstring& shortname_in) {
    std::wstring shortname = shortname_in;
    for (auto& ch : shortname) ch = (wchar_t)towlower(ch);
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr || shortname.empty()) return nullptr;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"StaticMesh") continue;
        const auto* fn = o->get_fname(); if (fn == nullptr) continue;
        std::wstring nm = fn->to_string(); for (auto& ch : nm) ch = (wchar_t)towlower(ch);
        if (nm == shortname) return o;
    }
    return nullptr;
}
API::UObject* sp_spawn_part(API::UObject* actor, API::UObject* mesh, API::UObject* parent, const API::FName& socket,
                            const double loc[3], const double rot[3], const double scl[3]) {
    auto* cls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.StaticMeshComponent");
    auto* part = cls ? API::get()->add_component_by_class(actor, cls, false) : nullptr;
    if (part == nullptr) return nullptr;
    { alignas(16) uint8_t p[64] = {0}; *reinterpret_cast<void**>(p) = mesh; part->call_function(L"SetStaticMesh", p); }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 0; part->call_function(L"SetCollisionEnabled", p); }
    { alignas(16) uint8_t p[64] = {0}; p[0] = g_cfg.slide_part_shadow ? 1 : 0; part->call_function(L"SetCastShadow", p); }
    if (parent != nullptr) {
        if (auto* fn = sc_fn(part, L"K2_AttachToComponent")) {
            alignas(16) uint8_t p[128] = {0}; bool ok = true; const uint8_t rule = 0; const bool weld = false;
            sm_put(fn, p, sizeof(p), L"Parent", &parent, sizeof(void*), &ok);
            sm_put(fn, p, sizeof(p), L"SocketName", &socket, sizeof(int32_t) * 2, &ok);
            sm_put(fn, p, sizeof(p), L"LocationRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"RotationRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"ScaleRule", &rule, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bWeldSimulatedBodies", &weld, 1, &ok);
            if (ok) fn->call(part, p);
        }
    }
    if (auto* fn = sc_fn(part, L"K2_SetRelativeLocationAndRotation")) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
        sm_put(fn, p, sizeof(p), L"NewLocation", loc, 24, &ok);
        sm_put(fn, p, sizeof(p), L"NewRotation", rot, 24, &ok);
        sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
        sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
        if (ok) fn->call(part, p);
    }
    { alignas(16) uint8_t q[64] = {0}; memcpy(q, scl, 24); part->call_function(L"SetRelativeScale3D", q); }
    return part;
}
// The rackable bone for this weapon stem, from the slide_bones table; slide_part_bone otherwise.
std::wstring sp_rack_bone_in(const char* table, const std::wstring& stem) {
    std::string tbl = trim_cfg(table);
    std::string lstem(stem.begin(), stem.end()); for (auto& ch : lstem) ch = (char)tolower((unsigned char)ch);
    size_t pos = 0;
    while (pos < tbl.size()) {
        size_t comma = tbl.find(',', pos); if (comma == std::string::npos) comma = tbl.size();
        std::string ent = tbl.substr(pos, comma - pos);
        const size_t colon = ent.find(':');
        if (colon != std::string::npos) {
            std::string w = ent.substr(0, colon), b = ent.substr(colon + 1);
            for (auto& ch : w) ch = (char)tolower((unsigned char)ch);
            while (!b.empty() && (unsigned char)b.back() <= ' ') b.pop_back();
            if (lstem.rfind(w, 0) == 0) return std::wstring(b.begin(), b.end());   // the table name is a PREFIX of the stem (SK_ShotgunCommon_Default -> shotguncommon)
        }
        pos = comma + 1;
    }
    return L"";
}
std::wstring sp_rack_bone_for(const std::wstring& stem) {
    std::wstring r = sp_rack_bone_in(g_cfg.slide_bones_override, stem);   // the cfg's per-weapon overrides first
    if (r.empty()) r = sp_rack_bone_in(g_cfg.slide_bones, stem);
    if (r.empty()) { const std::string d = trim_cfg(g_cfg.slide_part_bone); r.assign(d.begin(), d.end()); }
    return r;
}
// Every loaded StaticMesh named SM_<stem>_..., lowercased, for the per-bone match.
void sp_collect_weapon_meshes(const std::wstring& stem, std::vector<std::pair<std::wstring, API::UObject*>>& out) {
    std::wstring prefix = L"sm_" + stem + L"_"; for (auto& ch : prefix) ch = (wchar_t)towlower(ch);
    auto* arr = API::get()->get_uobject_array(); if (arr == nullptr) return;
    const int32_t nn = arr->get_object_count();
    for (int32_t i = 0; i < nn; ++i) {
        auto* o = static_cast<API::UObject*>(arr->get_object(i));
        if (o == nullptr || IsBadReadPtr(o, sizeof(void*))) continue;
        if (class_name_of(o) != L"StaticMesh") continue;
        const auto* fn = o->get_fname(); if (fn == nullptr) continue;
        std::wstring nm = fn->to_string(); for (auto& ch : nm) ch = (wchar_t)towlower(ch);
        if (nm.rfind(prefix, 0) != 0) continue;
        if (nm.find(L"shadow") != std::wstring::npos) continue;
        if (!g_cfg.slide_part_ui && (nm.find(L"_ui_") != std::wstring::npos || nm.rfind(prefix + L"ui_", 0) == 0)) continue;
        out.emplace_back(nm, o);
    }
}
bool sp_spawn(API::UObject* actor, API::UObject* src) {
    // The weapon stem from the asset name: SK_Magnum_Default -> Magnum.
    API::UObject* mesh = nullptr;
    for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
        auto** pm = src->get_property_data<API::UObject*>(nm);
        if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
    }
    if (mesh == nullptr || mesh->get_fname() == nullptr) return false;
    std::wstring asset = mesh->get_fname()->to_string();
    std::wstring stem = asset;
    if (stem.rfind(L"SK_", 0) == 0) stem = stem.substr(3);
    stem = weapon_stem_from_mesh(stem);
    API::UObject* parent = nullptr; API::FName socket{}; double loc[3] = {0}, rot[3] = {0}, scl[3] = {1, 1, 1};
    if (!sc_read_attach(src, &parent, &socket, loc, rot, scl)) return false;
    // The bind pose: a reference-pose skeletal copy, read once, destroyed.
    API::UObject* ref = nullptr;
    if (auto* scls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.SkeletalMeshComponent")) {
        ref = API::get()->add_component_by_class(actor, scls, false);
        if (ref != nullptr) {
            for (const wchar_t* fname : { L"SetSkinnedAssetAndUpdate", L"SetSkeletalMesh" }) {
                auto* fn = sc_fn(ref, fname); if (fn == nullptr) continue;
                alignas(16) uint8_t p[64] = {0}; bool ok = true; const bool reinit = true;
                sm_put(fn, p, sizeof(p), L"NewMesh", &mesh, sizeof(void*), &ok);
                sm_put(fn, p, sizeof(p), L"bReinitPose", &reinit, 1, &ok);
                if (ok) { fn->call(ref, p); break; }
            }
            sc_set_visibility(ref, false);
        }
    }
    const std::wstring want_slide = sp_rack_bone_for(stem);
    std::vector<std::pair<std::wstring, API::UObject*>> meshes;
    sp_collect_weapon_meshes(stem, meshes);
    std::vector<bool> claimed(meshes.size(), false);
    API::UObject* real_mat = g_cfg.slide_part_mat ? sp_get_material(src, 0) : nullptr;
    API::FName body_bone{}; XformD body_bind{}; bool have_body = false;
    int32_t nb = 0;
    if (auto* fn = sc_fn(src, L"GetNumBones")) { alignas(16) uint8_t p[64] = {0}; fn->call(src, p); auto* r = fn->find_property(L"ReturnValue"); if (r) nb = *reinterpret_cast<int32_t*>(p + r->get_offset()); }
    auto* gb = sc_fn(src, L"GetBoneName");
    std::wstring report;
    s_sp_count = 0;
    for (int32_t i = 0; gb != nullptr && i < nb && s_sp_count < 12; ++i) {
        alignas(16) uint8_t p[64] = {0}; bool ok = true;
        sm_put(gb, p, sizeof(p), L"BoneIndex", &i, sizeof(int32_t), &ok);
        auto* r = gb->find_property(L"ReturnValue");
        if (!ok || r == nullptr) break;
        gb->call(src, p);
        API::FName bn; memcpy(&bn, p + r->get_offset(), sizeof(int32_t) * 2);
        const std::wstring bname = bn.to_string();
        if (bname == L"World") continue;
        if (g_cfg.slide_part_hide != 2 && bname != want_slide) continue;   // every mode but the rebuild: the slide alone
        if (bname == want_slide) s_sp_slide_index = i;
        // Every shipped part that belongs to this bone. Names seen in the pak: SM_Magnum_Slide_Default
        // (suffix dropped), SM_AssaultRifle_GunBody_M_Default (suffix kept), SM_BattleRifle_Root_M_Default_Scope
        // (several parts per bone), SM_Shotgun_Glass_HandleJnt_M_Default (a prefixed extra).
        std::wstring bstem = bname;
        if (bstem.size() > 2 && (bstem.compare(bstem.size() - 2, 2, L"_M") == 0 || bstem.compare(bstem.size() - 2, 2, L"_L") == 0 || bstem.compare(bstem.size() - 2, 2, L"_R") == 0)) bstem = bstem.substr(0, bstem.size() - 2);
        std::wstring lb = bname, ls = bstem, lstem = stem;
        for (auto& ch : lb) ch = (wchar_t)towlower(ch);
        for (auto& ch : ls) ch = (wchar_t)towlower(ch);
        for (auto& ch : lstem) ch = (wchar_t)towlower(ch);
        int found_here = 0;
        for (auto& mp : meshes) {
            if (s_sp_count >= 12) break;
            const std::wstring& nm = mp.first;   // sm_<stem>_...
            const std::wstring rest = nm.substr(3 + lstem.size() + 1);
            bool match = false;
            for (const std::wstring& b : { lb, ls }) {
                if (rest == b + L"_default") match = true;
                else if (rest.rfind(b + L"_default_", 0) == 0) match = true;
                else if (rest == L"glass_" + b + L"_default" || rest == L"screen_" + b + L"_default") match = true;
            }
            if (!match) continue;
            // A stem match must not also be a longer bone's name (Root_M vs Root_M_Default_Body is
            // fine; Wing1Jnt vs Wing1Jnt_L is not): the bone's own full name wins where both exist.
            auto* part = sp_spawn_part(actor, mp.second, parent, socket, loc, rot, scl);
            if (part == nullptr) { report += bname + L":spawn-failed  "; continue; }
            claimed[&mp - &meshes[0]] = true;
            if (real_mat != nullptr) sp_set_material(part, 0, real_mat);
            SlidePart& sp = s_sp_parts[s_sp_count++];
            sp.comp.set(part); sp.bone = bn; sp.bone_name = bname; sp.mesh = mp.second; sp.dropped = false;
            if (g_cfg.slide_part_bind == 1) sp.have_bind = (ref != nullptr) && sp_socket_component(ref, bn, &sp.bind);
            else                            sp.have_bind = sp_socket_component(src, bn, &sp.bind);   // the live pose, now
            sp.is_slide = (bname == want_slide);
            sp.is_mag = (lb.find(L"magazine") != std::wstring::npos || lb.find(L"megazine") != std::wstring::npos);
            // Where the mesh's geometry sits relative to its pivot: UStaticMesh::GetBoundingBox.
            double cx = 0, cy = 0, cz = 0, ext = 0; bool have_box = false;
            if (auto* fn = sc_fn(mp.second, L"GetBoundingBox")) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* r = fn->find_property(L"ReturnValue");
                if (r != nullptr) {
                    fn->call(mp.second, p);
                    const double* b = reinterpret_cast<const double*>(p + r->get_offset());   // FBox: Min xyz, Max xyz
                    cx = 0.5 * (b[0] + b[3]); cy = 0.5 * (b[1] + b[4]); cz = 0.5 * (b[2] + b[5]);
                    ext = std::sqrt((b[3]-b[0])*(b[3]-b[0]) + (b[4]-b[1])*(b[4]-b[1]) + (b[5]-b[2])*(b[5]-b[2]));
                    have_box = true;
                }
            }
            const double bone_r = std::sqrt(sp.bind.tx*sp.bind.tx + sp.bind.ty*sp.bind.ty + sp.bind.tz*sp.bind.tz);
            const double centre_r = std::sqrt(cx*cx + cy*cy + cz*cz);
            // Authored around its bone: geometry centred near the pivot while the bone itself sits
            // well away from the mesh origin. Authored around the origin otherwise.
            const bool auto_bone = have_box && bone_r > 2.0 && centre_r < 0.5 * bone_r;
            sp.bone_pivot = (g_cfg.slide_part_pivot < 0) ? auto_bone : (g_cfg.slide_part_pivot == 1);
            wchar_t info[160];
            swprintf_s(info, L"%ls->%ls[box c=(%.1f %.1f %.1f) d=%.0f; bone r=%.1f; %ls]  ", bname.c_str(), mp.first.c_str(), cx, cy, cz, ext, bone_r,
                       sp.bone_pivot ? L"at bone" : L"by delta");
            report += info;
            if (lb.find(L"body") != std::wstring::npos && !have_body) { body_bone = bn; body_bind = sp.bind; have_body = sp.have_bind; }
            ++found_here;
        }
        if (found_here == 0) report += bname + L":none  ";
    }
    // Orphans: shipped parts no bone claimed (the magnum's Shroud is the gun's top). They ride
    // the body bone by delta, which is what a fixed part of the frame does.
    if (g_cfg.slide_part_orphans && have_body) {
        for (size_t i = 0; i < meshes.size() && s_sp_count < 12; ++i) {
            if (claimed[i]) continue;
            auto* part = sp_spawn_part(actor, meshes[i].second, parent, socket, loc, rot, scl);
            if (part == nullptr) continue;
            if (real_mat != nullptr) sp_set_material(part, 0, real_mat);
            SlidePart& sp = s_sp_parts[s_sp_count++];
            sp.comp.set(part); sp.bone = body_bone; sp.bone_name = L"(orphan on body)"; sp.bind = body_bind; sp.have_bind = true; sp.mesh = meshes[i].second; sp.dropped = false;
            sp.is_slide = false; sp.is_mag = false; sp.bone_pivot = false; sp.claimed = false;
            report += L"orphan " + meshes[i].first + L"->body  ";
        }
    }
    if (ref != nullptr) { alignas(16) uint8_t p[64] = {0}; ue_destroy_component(ref); }
    API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: %ls -> stem '%ls', rack bone '%ls', %d part(s) of %d loaded, rest from %s, material %s: %ls", asset.c_str(), stem.c_str(), want_slide.c_str(), s_sp_count, (int)meshes.size(),
                         g_cfg.slide_part_bind == 1 ? "the bind pose" : "the live pose", real_mat ? "the real gun's" : "each part's own", report.c_str());
    if (s_sp_count == 0) return false;
    s_sp_src.set(src);
    for (int i = 0; i < s_sp_count; ++i) if (s_sp_parts[i].is_slide) g_slide_rack_found = true;
    if (g_cfg.slide_part_hide == 2) {
        API::UObject* body_part = nullptr;
        for (int i = 0; i < s_sp_count; ++i) { std::wstring lb = s_sp_parts[i].bone_name; for (auto& ch : lb) ch = (wchar_t)towlower(ch); if (lb.find(L"body") != std::wstring::npos) { body_part = s_sp_parts[i].comp.get(); break; } }
        if (body_part == nullptr && s_sp_count > 0) body_part = s_sp_parts[0].comp.get();
        sp_kids_take(src, body_part);
        sc_set_scale(src, 0.001); s_sp_hidden = true;
    }
    else if (g_cfg.slide_part_hide == 1) { sp_hide_bone(src, true); s_sp_hidden = true; }
    else if (g_cfg.slide_part_hide == 5) {
        XformD slc{}; sp_socket_component(src, sp_slide_bone_name(), &slc);
        sp_hide_bone(src, true); s_sp_hidden = true;
        sp_find_bone_arrays(src, nb, s_sp_slide_index, slc);
    }
    else if (g_cfg.slide_part_hide == 6) { sp_log_morphs(src); }
    sp_log_materials(src);
    if (g_cfg.slide_part_hide == 4) {
        const int slot = (g_cfg.slide_hide_mat_slot >= 0) ? g_cfg.slide_hide_mat_slot : (g_cfg.slide_hide_section >= 0 ? g_cfg.slide_hide_section : 0);
        auto* mat = sp_find_material(trim_cfg(g_cfg.slide_hide_mat));
        if (mat == nullptr) API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: no material matching '%s' loaded; slot %d left as is", trim_cfg(g_cfg.slide_hide_mat).c_str(), slot);
        else {
            s_sp_mat_orig.set(sp_get_material(src, slot)); s_sp_mat_slot = slot;
            sp_set_material(src, slot, mat);
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: slot %d material -> %ls", slot, mat->get_full_name().c_str());
        }
    }
    // Is the arms rig's asset Nanite too? The arms bone-hide works; if the arms are not Nanite,
    // that is the difference, not the call.
    if (auto* rig = rig_tracked_component()) {
        for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
            auto** pm = rig->get_property_data<API::UObject*>(nm);
            if (pm == nullptr || IsBadReadPtr(pm, sizeof(void*)) || *pm == nullptr) continue;
            int nan = -1;
            if (auto* pr = (*pm)->get_class()->find_property(L"NaniteSettings")) { const uint8_t* q = reinterpret_cast<const uint8_t*>(*pm) + pr->get_offset(); if (!IsBadReadPtr(q, 1)) nan = q[0] & 1; }
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: arms asset %ls NaniteSettings.bEnabled=%d", (*pm)->get_fname() ? (*pm)->get_fname()->to_string().c_str() : L"?", nan);
            break;
        }
    }
    return true;
}
// A physics box proxy for a dropped mesh component that has no collision of its own: an
// invisible BoxComponent from the mesh's world bounds simulates, the mesh rides it.
API::UObject* sp_drop_proxy_for(API::UObject* m) {
    double origin[3] = {0, 0, 0}, extent[3] = {2, 2, 4};
    if (auto* fn = sc_fn(m, L"K2_GetComponentBounds")) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
        auto* po = fn->find_property(L"Origin"); auto* pe = fn->find_property(L"BoxExtent");
        if (po && pe) { fn->call(m, p); memcpy(origin, p + po->get_offset(), 24); memcpy(extent, p + pe->get_offset(), 24); }
    }
    auto* bcls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.BoxComponent");
    auto* actor = fp_weapon_actor();
    auto* box = (bcls && actor) ? API::get()->add_component_by_class(actor, bcls, false) : nullptr;
    if (box == nullptr) return nullptr;
    { alignas(16) uint8_t p[64] = {0}; p[0] = 1; p[1] = 1; p[2] = 1; p[3] = 0; box->call_function(L"K2_DetachFromComponent", p); }
    { alignas(16) uint8_t p[64] = {0}; auto* d = reinterpret_cast<double*>(p); d[0] = std::fmax(1.0, extent[0]); d[1] = std::fmax(1.0, extent[1]); d[2] = std::fmax(1.0, extent[2]); p[24] = 1; box->call_function(L"SetBoxExtent", p); }
    XformD cw{};
    if (sc_component_world(m, &cw)) {
        XformD bw = cw; bw.tx = origin[0]; bw.ty = origin[1]; bw.tz = origin[2]; bw.sx = bw.sy = bw.sz = 1.0;
        if (auto* fn = sc_fn(box, L"K2_SetWorldTransform")) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
            sm_put(fn, p, sizeof(p), L"NewTransform", &bw, sizeof(XformD), &ok);
            sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
            if (ok) fn->call(box, p);
        }
    }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 1; box->call_function(L"SetHiddenInGame", p); }
    sp_attach(m, box, make_fname(L"None"), 1);   // KeepWorld
    { alignas(16) uint8_t p[64] = {0}; p[0] = 0; m->call_function(L"SetCollisionEnabled", p); }
    { alignas(16) uint8_t p[64] = {0}; API::FName nm = make_fname(L"PhysicsActor"); memcpy(p, &nm, sizeof(int32_t) * 2); p[8] = 1; box->call_function(L"SetCollisionProfileName", p); }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 3; box->call_function(L"SetCollisionEnabled", p); }
    { alignas(16) uint8_t p[64] = {0}; p[0] = 1; box->call_function(L"SetSimulatePhysics", p); }
    return box;
}
// ---- NATIVE PARTS (mode 7). The game's own rack component, found among the real mesh's
// children by the mesh it renders; its socket keeps every game animation, our offset rides on it.
struct NpPart { TrackedObject comp; double orig[3]; double orig_rot[3]; API::FName bone; };
NpPart s_np_parts[4]; int s_np_count = 0; bool s_np_rot = false;
TrackedObject s_np_slide, s_np_src; API::FName s_np_bone; std::wstring s_np_bone_w; std::wstring s_np_stem; double s_np_orig[3] = {0, 0, 0}; bool s_np_have = false;
bool s_np_quiet = false;   // slide_part_tick: the listing line is off after the second failed try
bool np_spawn(API::UObject* src) {
    s_np_have = false; s_np_slide = TrackedObject{};
    API::UObject* mesh = nullptr;
    for (const wchar_t* nm : { L"SkinnedAsset", L"SkeletalMesh" }) {
        auto** pm = src->get_property_data<API::UObject*>(nm);
        if (pm != nullptr && !IsBadReadPtr(pm, sizeof(void*)) && *pm != nullptr) { mesh = *pm; break; }
    }
    if (mesh == nullptr || mesh->get_fname() == nullptr) return false;
    std::wstring stem = mesh->get_fname()->to_string();
    if (stem.rfind(L"SK_", 0) == 0) stem = stem.substr(3);
    stem = weapon_stem_from_mesh(stem);
    // The spec: "Bone" | "BoneA+BoneB" | "Bone@rot" | "Bone@back=0.03". Several parts move
    // together; @rot hinges; @back pushes this weapon's grab zone back along the barrel (metres).
    std::wstring spec = sp_rack_bone_for(stem);
    s_np_rot = false;
    g_sl_zone_back.store(0.0f, std::memory_order_relaxed); g_sl_zone_up.store(0.0f, std::memory_order_relaxed); g_sl_zone_right.store(0.0f, std::memory_order_relaxed); g_sl_zone_reload_only.store(false, std::memory_order_relaxed); g_sl_zone_every_shot.store(false, std::memory_order_relaxed); g_sl_zone_pump.store(false, std::memory_order_relaxed); g_sl_zone_pull_down.store(false, std::memory_order_relaxed); g_sl_insert.store(-1.0f, std::memory_order_relaxed);
    {
        size_t at = spec.find(L'@');
        std::wstring opts = (at != std::wstring::npos) ? spec.substr(at + 1) : L"";
        if (at != std::wstring::npos) spec = spec.substr(0, at);
        size_t pos = 0;
        while (pos < opts.size()) {
            size_t nx = opts.find(L'@', pos); if (nx == std::wstring::npos) nx = opts.size();
            std::wstring o = opts.substr(pos, nx - pos);
            if (o == L"rot") s_np_rot = true;
            else if (o == L"reloadonly") g_sl_zone_reload_only.store(true, std::memory_order_relaxed);
            else if (o == L"everyshot") g_sl_zone_every_shot.store(true, std::memory_order_relaxed);
            else if (o == L"pump") g_sl_zone_pump.store(true, std::memory_order_relaxed);
            else if (o == L"pulldown") g_sl_zone_pull_down.store(true, std::memory_order_relaxed);
            else if (o.rfind(L"insert=", 0) == 0) g_sl_insert.store((float)_wtof(o.c_str() + 7), std::memory_order_relaxed);
            else if (o.rfind(L"back=", 0) == 0) g_sl_zone_back.store((float)_wtof(o.c_str() + 5), std::memory_order_relaxed);
            else if (o.rfind(L"up=", 0) == 0) g_sl_zone_up.store((float)_wtof(o.c_str() + 3), std::memory_order_relaxed);
            else if (o.rfind(L"right=", 0) == 0) g_sl_zone_right.store((float)_wtof(o.c_str() + 6), std::memory_order_relaxed);
            pos = nx + 1;
        }
    }
    std::vector<std::wstring> bones;
    { size_t pos = 0; while (pos <= spec.size()) { size_t plus = spec.find(L'+', pos); if (plus == std::wstring::npos) plus = spec.size(); if (plus > pos) bones.push_back(spec.substr(pos, plus - pos)); if (plus >= spec.size()) break; pos = plus + 1; } }
    const std::wstring bone = bones.empty() ? spec : bones[0];
    std::wstring listing;
    for (int i = 0; i < 4; ++i) s_np_parts[i] = NpPart{};
    s_np_count = 0;
    API::UObject* hit = nullptr; std::wstring hitmesh;
    weapon_components([&](API::UObject* c) {
        auto** pp = c->get_property_data<API::UObject*>(L"AttachParent");
        if (pp == nullptr || IsBadReadPtr(pp, sizeof(void*)) || *pp != src) return true;
        auto** pm = c->get_property_data<API::UObject*>(L"StaticMesh");
        if (pm == nullptr || IsBadReadPtr(pm, sizeof(void*)) || *pm == nullptr || (*pm)->get_fname() == nullptr) return true;
        std::wstring mn = (*pm)->get_fname()->to_string(); std::wstring lo = mn; for (auto& ch : lo) ch = (wchar_t)towlower(ch);
        listing += mn + L"  ";
        for (const std::wstring& b : bones) {
            std::wstring bs = b;
            if (bs.size() > 2 && (bs.compare(bs.size() - 2, 2, L"_M") == 0 || bs.compare(bs.size() - 2, 2, L"_L") == 0 || bs.compare(bs.size() - 2, 2, L"_R") == 0)) bs = bs.substr(0, bs.size() - 2);
            std::wstring w1 = L"_" + bs + L"_", w2 = L"_" + b + L"_";
            for (auto& ch : w1) ch = (wchar_t)towlower(ch);
            for (auto& ch : w2) ch = (wchar_t)towlower(ch);
            if ((lo.find(w1) != std::wstring::npos || lo.find(w2) != std::wstring::npos) && s_np_count < 4) {
                NpPart& np = s_np_parts[s_np_count++];
                np.comp.set(c); np.bone = make_fname(b.c_str());
                if (auto* pl = c->get_property_data<double>(L"RelativeLocation")) if (!IsBadReadPtr(pl, 24)) { np.orig[0] = pl[0]; np.orig[1] = pl[1]; np.orig[2] = pl[2]; }
                if (auto* pr = c->get_property_data<double>(L"RelativeRotation")) if (!IsBadReadPtr(pr, 24)) { np.orig_rot[0] = pr[0]; np.orig_rot[1] = pr[1]; np.orig_rot[2] = pr[2]; }
                if (hit == nullptr) { hit = c; hitmesh = mn; }
                break;
            }
        }
        return true;
    });
    if (!s_np_quiet || hit != nullptr) API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART native: %ls parts on the real mesh: %ls-> rack '%ls' = %ls (%d part(s)%s)", stem.c_str(), listing.c_str(), spec.c_str(), hit ? hitmesh.c_str() : L"NOT FOUND", s_np_count, s_np_rot ? ", hinge" : "");
    if (hit == nullptr) return false;
    s_np_orig[0] = s_np_parts[0].orig[0]; s_np_orig[1] = s_np_parts[0].orig[1]; s_np_orig[2] = s_np_parts[0].orig[2];
    s_np_slide.set(hit); s_np_src.set(src); s_np_bone = make_fname(bone.c_str()); s_np_bone_w = bone; s_np_stem = stem; s_np_have = true;
    g_slide_rack_found = true;
    return true;
}
void np_set_rel(API::UObject* c, const double loc[3], const double rot[3]) {
    if (auto* fn = sc_fn(c, L"K2_SetRelativeLocationAndRotation")) {
        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
        sm_put(fn, p, sizeof(p), L"NewLocation", loc, 24, &ok);
        sm_put(fn, p, sizeof(p), L"NewRotation", rot, 24, &ok);
        sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
        sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
        if (ok) fn->call(c, p);
    }
}
void np_teardown() {
    g_sl_zone_back.store(0.0f, std::memory_order_relaxed); g_sl_zone_up.store(0.0f, std::memory_order_relaxed); g_sl_zone_right.store(0.0f, std::memory_order_relaxed); g_sl_zone_reload_only.store(false, std::memory_order_relaxed); g_sl_zone_every_shot.store(false, std::memory_order_relaxed); g_sl_zone_pump.store(false, std::memory_order_relaxed); g_sl_zone_pull_down.store(false, std::memory_order_relaxed); g_sl_insert.store(-1.0f, std::memory_order_relaxed); g_slide_zone_hot.store(false, std::memory_order_relaxed);
    for (int i = 0; i < s_np_count; ++i) { if (auto* c = s_np_parts[i].comp.get()) np_set_rel(c, s_np_parts[i].orig, s_np_parts[i].orig_rot); s_np_parts[i] = NpPart{}; }
    s_np_count = 0;
    s_np_slide = TrackedObject{}; s_np_src = TrackedObject{}; s_np_have = false;
    g_slide_rack_found = false;
    g_sl_part_valid.store(false, std::memory_order_relaxed);
}
void np_tick() {
    auto* c = s_np_slide.get();
    if (c == nullptr) return;
    const float pull_cm = g_slide_pull.load(std::memory_order_relaxed) * 304.8f;
    const double along = -(double)(pull_cm * g_cfg.slide_part_sign) - (double)g_cfg.slide_part_test;
    double gun_axis[3] = {0, 0, 0}; gun_axis[g_cfg.slide_part_axis] = along;   // in the gun mesh's frame
    for (int i = 0; i < s_np_count; ++i) {
        auto* pc = s_np_parts[i].comp.get();
        if (pc == nullptr) continue;
        double loc[3] = {s_np_parts[i].orig[0], s_np_parts[i].orig[1], s_np_parts[i].orig[2]};
        double rot[3] = {s_np_parts[i].orig_rot[0], s_np_parts[i].orig_rot[1], s_np_parts[i].orig_rot[2]};
        if (s_np_rot) {
            // A HINGE (the rocket launcher's clamp): on the reload press it swings OPEN and the
            // lock holds it there after the seat; the rack closes it (pull fraction 0 -> 1 = open
            // -> closed). Outside a reload the pull just swings it by degrees per cm.
            const float frac = (g_cfg.slide_travel > 0.0f) ? std::fmin(1.0f, std::fmax(0.0f, g_slide_pull.load(std::memory_order_relaxed) / g_cfg.slide_travel)) : 0.0f;
            const bool reloading = (s_reload != ReloadState::Idle) || s_sl_lock_pending;
            rot[g_cfg.slide_part_rot_axis] += reloading ? (double)g_cfg.slide_part_open_deg * (1.0 - (double)frac)
                                                        : (double)(pull_cm + g_cfg.slide_part_test) * (double)g_cfg.slide_part_rot_deg;
        } else {
            double ax[3] = {gun_axis[0], gun_axis[1], gun_axis[2]};
            if (g_cfg.slide_part_frame == 1) {
                // The gun-frame axis into THIS part's own socket frame (the spiker's pin socket is
                // not the hammer's: converted through the hammer's it went down, not back).
                XformD st{};
                if (auto* src = s_np_src.get()) {
                    if (sp_socket_component(src, s_np_parts[i].bone, &st)) {
                        const QD q{st.qx, st.qy, st.qz, st.qw};
                        double l[3]; qd_rot(qd_conj(q), ax, l);
                        ax[0] = l[0]; ax[1] = l[1]; ax[2] = l[2];
                    }
                }
            }
            loc[0] += ax[0]; loc[1] += ax[1]; loc[2] += ax[2];
        }
        np_set_rel(pc, loc, rot);
    }
    // The grab zone: three reference points (Config.hpp slide_zone), all three logged.
    {
        // By-name calls (the class find_function path came back empty on the shotgun and the
        // launcher, 2026-09-06): Origin is K2_GetComponentBounds' first out-param, at offset 0.
        Vec3 bounds{}, sock{}, comp{}; bool hb = false, hs = false, hc = false;
        const char* how = "";
        hb = part_world_centre(c, &bounds, &how);
        if (auto* src = s_np_src.get()) hs = call_socket_location(src, s_np_bone_w.c_str(), &sock);
        hc = call_ret_vec3(c, L"K2_GetComponentLocation", &comp);
        Vec3 use{}; bool have = false;
        if (g_cfg.slide_zone == 2 && hs) { use = sock; have = true; }
        else if (g_cfg.slide_zone == 3 && hc) { use = comp; have = true; }
        else if (hb) { use = bounds; have = true; }
        if (have) { g_sl_part_x.store(use.x, std::memory_order_relaxed); g_sl_part_y.store(use.y, std::memory_order_relaxed); g_sl_part_z.store(use.z, std::memory_order_relaxed); }
        g_sl_part_valid.store(have, std::memory_order_relaxed);
        if (g_cfg.slide_log) {
            static uint32_t s_n = 0;
            if ((s_n++ % 30u) == 0u) {
                Vec3 gun{}; const bool hg = (s_np_src.get() != nullptr) && call_ret_vec3(s_np_src.get(), L"K2_GetComponentLocation", &gun);
                API::get()->log_info("[Halo-CampE-UEVR] SLIDEZONE %ls/%ls refs (cm, world): bounds[%s] %s(%.0f %.0f %.0f) socket %s(%.0f %.0f %.0f) comp %s(%.0f %.0f %.0f) gun %s(%.0f %.0f %.0f) using mode %d",
                                     s_np_stem.c_str(), s_np_bone_w.c_str(), how, hb ? "" : "-", bounds.x, bounds.y, bounds.z, hs ? "" : "-", sock.x, sock.y, sock.z, hc ? "" : "-", comp.x, comp.y, comp.z, hg ? "" : "-", gun.x, gun.y, gun.z, g_cfg.slide_zone);
            }
        }
    }
}
// THE MAGAZINE PART DROPS. Called on the reload press: the gun's own mag part detaches (keep
// world), takes a physics profile and falls. Returns true when it handled the drop, so the
// copy drop (of the game's hidden magazine mesh) stands down. A fresh part comes on the seat.
bool slide_parts_mag_drop() {
    if (!g_cfg.slide_part_magdrop || s_sp_count == 0) return false;
    for (int i = 0; i < s_sp_count; ++i) {
        SlidePart& sp = s_sp_parts[i];
        auto* m = sp.comp.get();
        if (!sp.is_mag || m == nullptr) continue;
        if (auto* old = s_sp_dropped.get()) ue_destroy_component(old);
        if (auto* old = s_sp_dropped_proxy.get()) ue_destroy_component(old);
        s_sp_dropped = TrackedObject{}; s_sp_dropped_proxy = TrackedObject{};
        // The asset's collision shapes, as a fact: UStaticMesh::BodySetup -> AggGeom counts.
        int nsph = -1, nbox = -1, ncap = -1, ncvx = -1;
        if (sp.mesh != nullptr) {
            if (auto** pbs = sp.mesh->get_property_data<API::UObject*>(L"BodySetup")) {
                if (!IsBadReadPtr(pbs, sizeof(void*)) && *pbs != nullptr) {
                    auto* bs = *pbs;
                    if (auto* pag = bs->get_class()->find_property(L"AggGeom")) {
                        const uint8_t* ag = reinterpret_cast<const uint8_t*>(bs) + pag->get_offset();
                        // FKAggregateGeom: TArray SphereElems, BoxElems, SphylElems, ConvexElems, ... (num at +8 of each 16-byte header)
                        if (!IsBadReadPtr(ag, 64)) { nsph = *reinterpret_cast<const int32_t*>(ag + 8); nbox = *reinterpret_cast<const int32_t*>(ag + 24); ncap = *reinterpret_cast<const int32_t*>(ag + 40); ncvx = *reinterpret_cast<const int32_t*>(ag + 56); }
                    }
                }
            }
        }
        const int mode = g_cfg.slide_part_dropmode;
        { alignas(16) uint8_t p[64] = {0}; p[0] = 1; p[1] = 1; p[2] = 1; p[3] = 0; m->call_function(L"K2_DetachFromComponent", p); }
        API::UObject* sim = m;   // the component that simulates
        if (mode == 1) {
            // A box proxy from the part's world bounds; the part rides it.
            XformD cw{}; double origin[3] = {0, 0, 0}, extent[3] = {2, 2, 4};
            if (auto* fn = sc_fn(m, L"K2_GetComponentBounds")) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* po = fn->find_property(L"Origin"); auto* pe = fn->find_property(L"BoxExtent");
                if (po && pe) { fn->call(m, p); memcpy(origin, p + po->get_offset(), 24); memcpy(extent, p + pe->get_offset(), 24); }
            }
            auto* bcls = API::get()->find_uobject<API::UClass>(L"Class /Script/Engine.BoxComponent");
            auto* actor = fp_weapon_actor();
            auto* box = (bcls && actor) ? API::get()->add_component_by_class(actor, bcls, false) : nullptr;
            if (box != nullptr) {
                { alignas(16) uint8_t p[64] = {0}; p[0] = 1; p[1] = 1; p[2] = 1; p[3] = 0; box->call_function(L"K2_DetachFromComponent", p); }
                { alignas(16) uint8_t p[64] = {0}; auto* d = reinterpret_cast<double*>(p); d[0] = std::fmax(1.0, extent[0]); d[1] = std::fmax(1.0, extent[1]); d[2] = std::fmax(1.0, extent[2]); p[24] = 1; box->call_function(L"SetBoxExtent", p); }
                if (sc_component_world(m, &cw)) {
                    XformD bw = cw; bw.tx = origin[0]; bw.ty = origin[1]; bw.tz = origin[2]; bw.sx = bw.sy = bw.sz = 1.0;
                    if (auto* fn = sc_fn(box, L"K2_SetWorldTransform")) {
                        alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
                        sm_put(fn, p, sizeof(p), L"NewTransform", &bw, sizeof(XformD), &ok);
                        sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
                        sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
                        if (ok) fn->call(box, p);
                    }
                }
                { alignas(16) uint8_t p[64] = {0}; box->call_function(L"SetHiddenInGame", p); p[0] = 1; box->call_function(L"SetHiddenInGame", p); }
                // The part rides the box, keeping its world placement.
                if (auto* fn = sc_fn(m, L"K2_AttachToComponent")) {
                    alignas(16) uint8_t p[128] = {0}; bool ok = true; const uint8_t keep = 1; const bool weld = false; API::FName none = make_fname(L"None");
                    sm_put(fn, p, sizeof(p), L"Parent", &box, sizeof(void*), &ok);
                    sm_put(fn, p, sizeof(p), L"SocketName", &none, sizeof(int32_t) * 2, &ok);
                    sm_put(fn, p, sizeof(p), L"LocationRule", &keep, 1, &ok);
                    sm_put(fn, p, sizeof(p), L"RotationRule", &keep, 1, &ok);
                    sm_put(fn, p, sizeof(p), L"ScaleRule", &keep, 1, &ok);
                    sm_put(fn, p, sizeof(p), L"bWeldSimulatedBodies", &weld, 1, &ok);
                    if (ok) fn->call(m, p);
                }
                { alignas(16) uint8_t p[64] = {0}; p[0] = 0; m->call_function(L"SetCollisionEnabled", p); }
                sim = box; s_sp_dropped_proxy.set(box);
            }
        }
        { alignas(16) uint8_t p[64] = {0}; API::FName nm = make_fname(L"PhysicsActor"); memcpy(p, &nm, sizeof(int32_t) * 2); p[8] = 1; sim->call_function(L"SetCollisionProfileName", p); }
        if (mode == 2) {
            { alignas(16) uint8_t p[64] = {0}; p[0] = 5; sim->call_function(L"SetCollisionObjectType", p); }   // ECC_PhysicsBody
            { alignas(16) uint8_t p[64] = {0}; p[0] = 2; sim->call_function(L"SetCollisionResponseToAllChannels", p); }   // ECR_Block
        }
        { alignas(16) uint8_t p[64] = {0}; p[0] = 3; sim->call_function(L"SetCollisionEnabled", p); }
        { alignas(16) uint8_t p[64] = {0}; p[0] = 1; sim->call_function(L"SetSimulatePhysics", p); }
        int simulating = -1;
        if (auto* fn = sc_fn(sim, L"IsSimulatingPhysics")) {
            alignas(16) uint8_t p[64] = {0}; auto* r = fn->find_property(L"ReturnValue");
            if (r) { fn->call(sim, p); simulating = p[r->get_offset()] ? 1 : 0; }
        }
        s_sp_dropped.set(m); s_sp_dropped_at = now_ticks();
        sp.comp = TrackedObject{}; sp.dropped = true;
        if (g_cfg.reload_vr_log || g_cfg.slide_log)
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: magazine part dropped (mode %d, %s simulating=%d; asset collision: %d spheres %d boxes %d capsules %d convex)",
                                 mode, sim == m ? "the part" : "a box proxy", simulating, nsph, nbox, ncap, ncvx);
        return true;
    }
    return false;
}
void slide_part_tick() {
    auto* src = reload_weapon_default_comp();
    auto* actor = fp_weapon_actor();
    const bool usable = g_cfg.slide_part && reload_rack_available() && src != nullptr && actor != nullptr && slide_weapon_ok();
    const std::string key = weapon_key();
    static int s_bind_mode = -1; static int s_shadow_mode = -1; static int s_pivot_mode = -2; static int s_mat_mode = -1; static int s_orphan_mode = -1; static int s_kids_mode = -1; static int s_ui_mode = -1;
    const bool mode_changed = (g_cfg.slide_part_hide != s_sp_mode) || (g_cfg.slide_part_bind != s_bind_mode) || ((int)g_cfg.slide_part_shadow != s_shadow_mode) || (g_cfg.slide_part_pivot != s_pivot_mode)
                           || ((int)g_cfg.slide_part_mat != s_mat_mode) || ((int)g_cfg.slide_part_orphans != s_orphan_mode)
                           || (g_cfg.slide_part_kids != s_kids_mode) || ((int)g_cfg.slide_part_ui != s_ui_mode);
    static std::string s_np_key; static API::UObject* s_np_src = nullptr; static long long s_np_retry = 0;
    if (g_cfg.slide_part_hide == 7 || s_np_have) {
        if (!usable || g_cfg.slide_part_hide != 7 || (s_np_have && (key != s_np_key || s_np_src != src))) {
            if (s_np_have) np_teardown();
            if (s_sp_count > 0) sp_teardown("native mode");
            if (!usable || g_cfg.slide_part_hide != 7) { if (!usable) return; }
        }
        if (g_cfg.slide_part_hide == 7) {
            if (!s_np_have) {
                static std::string s_np_fail_key; static int s_np_fails = 0;
                // A fresh weapon actor attaches its parts some frames after the swap (logged: only a
                // light cone at the swap tick, the rack found on the 1.5 s retry). The seat's lock-back
                // and the phantom both require the part to be found, so a seat inside that window
                // skipped the rack of a gun that came back empty. Retry fast first.
                if (key != s_np_fail_key) { s_np_fail_key = key; s_np_fails = 0; s_np_retry = 0; }
                if (now_ticks() - s_np_retry < ms_to_ticks(s_np_fails < 8 ? 150 : (s_np_fails < 11 ? 1500 : 15000))) return;
                s_np_retry = now_ticks();
                s_np_quiet = s_np_fails >= 2;
                if (!np_spawn(src)) { ++s_np_fails; return; }
                s_np_fails = 0;
                s_np_key = key; s_np_src = src; s_sp_mode = 7;
            }
            np_tick();
            return;
        }
    }
    if (!usable || (s_sp_count > 0 && (key != s_sp_key || s_sp_src.get() != src || mode_changed))) {
        sp_teardown(!usable ? "unavailable" : (mode_changed ? "mode changed" : "weapon changed"));
        if (!usable) return;
    }
    if (s_sp_count == 0) {
        if (now_ticks() - s_sp_retry_at < ms_to_ticks(1500)) return;
        s_sp_retry_at = now_ticks();
        if (!sp_spawn(actor, src)) return;
        s_sp_key = key; s_sp_mode = g_cfg.slide_part_hide; s_bind_mode = g_cfg.slide_part_bind; s_shadow_mode = (int)g_cfg.slide_part_shadow; s_pivot_mode = g_cfg.slide_part_pivot;
        s_mat_mode = (int)g_cfg.slide_part_mat; s_orphan_mode = (int)g_cfg.slide_part_orphans; s_kids_mode = g_cfg.slide_part_kids; s_ui_mode = (int)g_cfg.slide_part_ui;
    }
    ++s_sp_ticks;
    // The dropped magazine part is removed after mag_drop_ms; a dropped slot re-spawns on the seat.
    if (s_sp_dropped.get() != nullptr && now_ticks() - s_sp_dropped_at > ms_to_ticks(g_cfg.mag_drop_ms)) {
        ue_destroy_component(s_sp_dropped.get()); s_sp_dropped = TrackedObject{};
        if (auto* px = s_sp_dropped_proxy.get()) ue_destroy_component(px);
        s_sp_dropped_proxy = TrackedObject{};
    }
    if (s_reload == ReloadState::Idle) {
        for (int i = 0; i < s_sp_count; ++i) {
            SlidePart& sp = s_sp_parts[i];
            if (!sp.dropped || sp.mesh == nullptr) continue;
            API::UObject* parent = nullptr; API::FName socket{}; double loc[3] = {0}, rot[3] = {0}, scl[3] = {1, 1, 1};
            if (!sc_read_attach(src, &parent, &socket, loc, rot, scl)) break;
            auto* part = sp_spawn_part(actor, sp.mesh, parent, socket, loc, rot, scl);
            if (part == nullptr) break;
            if (g_cfg.slide_part_mat) { if (auto* rm = sp_get_material(src, 0)) sp_set_material(part, 0, rm); }
            sp.comp.set(part); sp.dropped = false;
            if (g_cfg.reload_vr_log || g_cfg.slide_log) API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: magazine part re-spawned on the seat");
        }
    }
    if (s_sp_hidden) {
        if (g_cfg.slide_part_hide == 2) sc_set_scale(src, 0.001);
        else if (g_cfg.slide_part_hide == 1) {
            sp_hide_bone(src, true);
            if (g_cfg.slide_log && (s_sp_ticks % 90u) == 5u)
                API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: engine says %s hidden = %d", trim_cfg(g_cfg.slide_part_bone).c_str(), sp_is_bone_hidden(src));
        }
    }
    if (g_cfg.slide_part_hide == 5) { sp_hide_bone(src, true); sp_park_far_tick(src); }
    if (g_cfg.slide_part_hide == 6) sp_morph_tick(src);
    // Section hide (mode 3): the live-stepped material index; the previous one is shown again.
    if (g_cfg.slide_part_hide == 3) {
        const int want = g_cfg.slide_hide_section;
        if (want != s_sp_section_hidden) {
            if (s_sp_section_hidden >= 0) sp_show_section(src, s_sp_section_hidden, true);
            if (want >= 0) sp_show_section(src, want, false);
            s_sp_section_hidden = want;
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: material section %d hidden (previous shown)", want);
        }
    }
    const float pull_cm = g_slide_pull.load(std::memory_order_relaxed) * 304.8f;
    const double along = -(double)(pull_cm * g_cfg.slide_part_sign) - (double)g_cfg.slide_part_test;
    const bool mag_out = (s_reload != ReloadState::Idle);
    for (int i = 0; i < s_sp_count; ++i) {
        SlidePart& sp = s_sp_parts[i];
        auto* part = sp.comp.get();
        if (part == nullptr) continue;
        XformD now{};
        if (!sp_socket_component(src, sp.bone, &now)) continue;
        // D = now x inverse(bind), in the mesh's frame: rotation qn*conj(qb), translation tn - D*tb.
        XformD d = now;
        if (sp.bone_pivot) {
            // Authored around its bone: the part IS the bone. now already is the bone in the mesh frame.
        } else if (sp.have_bind) {
            const QD qn{now.qx, now.qy, now.qz, now.qw}, qb{sp.bind.qx, sp.bind.qy, sp.bind.qz, sp.bind.qw};
            const QD qd = qd_mul(qn, qd_conj(qb));
            const double tb[3] = {sp.bind.tx, sp.bind.ty, sp.bind.tz}; double tbr[3]; qd_rot(qd, tb, tbr);
            d.qx = qd.x; d.qy = qd.y; d.qz = qd.z; d.qw = qd.w;
            d.tx = now.tx - tbr[0]; d.ty = now.ty - tbr[1]; d.tz = now.tz - tbr[2];
        } else {
            d.qx = 0; d.qy = 0; d.qz = 0; d.qw = 1; d.tx = 0; d.ty = 0; d.tz = 0;
        }
        d.sx = 1.0; d.sy = 1.0; d.sz = 1.0;
        if (sp.is_slide) {
            double ax[3] = {0, 0, 0}; ax[g_cfg.slide_part_axis] = 1.0;
            d.tx += ax[0] * along; d.ty += ax[1] * along; d.tz += ax[2] * along;
        }
        if (auto* fn = sc_fn(part, L"K2_SetRelativeTransform")) {
            alignas(16) uint8_t p[RIG_PARAM_BUF] = {0}; bool ok = true; const bool no = false, yes = true;
            sm_put(fn, p, sizeof(p), L"NewTransform", &d, sizeof(XformD), &ok);
            sm_put(fn, p, sizeof(p), L"bSweep", &no, 1, &ok);
            sm_put(fn, p, sizeof(p), L"bTeleport", &yes, 1, &ok);
            if (ok) fn->call(part, p);
        }
        if (sp.is_mag) sc_set_visibility(part, !(mag_out && g_cfg.slide_part_maghide));
        if (sp.is_slide) {
            bool published = false;
            if (auto* fn = sc_fn(part, L"K2_GetComponentBounds")) {
                alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
                auto* po = fn->find_property(L"Origin");
                if (po != nullptr) {
                    fn->call(part, p);
                    const double* o = reinterpret_cast<const double*>(p + po->get_offset());
                    g_sl_part_x.store((float)o[0], std::memory_order_relaxed);
                    g_sl_part_y.store((float)o[1], std::memory_order_relaxed);
                    g_sl_part_z.store((float)o[2], std::memory_order_relaxed);
                    published = true;
                }
            }
            g_sl_part_valid.store(published, std::memory_order_relaxed);
        }
        if (g_cfg.slide_log && sp.is_slide && (s_sp_ticks % 90u) == 1u)
            API::get()->log_info("[Halo-CampE-UEVR] SLIDEPART: %ls now (%.2f %.2f %.2f) bind (%.2f %.2f %.2f) -> rel (%.2f %.2f %.2f) along %.2f",
                                 sp.bone_name.c_str(), now.tx, now.ty, now.tz, sp.bind.tx, sp.bind.ty, sp.bind.tz, d.tx, d.ty, d.tz, along);
    }
}

// ================================================================ THE SNAPSHOT'S OWN READS
//
// THE RACK PART'S WORLD CENTRE, READ IN THE SNAPSHOT. g_sl_part_* is published by slide_part_tick,
// which reload_engine_ticks runs AFTER slide_update consumed it, so the zone was always built on a
// position one whole engine tick old. This is the same reference choice (slide_zone: 2 = the
// socket, 3 = the component's own location, otherwise the part's bounds centre) read at the
// snapshot's instant, so the zone and the hand it is compared against are the same moment.
// *key is the component the value came from, which is what mode 3's learn-and-hold is keyed on:
// a weapon swap brings a different component and re-learns.
bool sl_part_world_now(Vec3* out, void** key) {
    if (key != nullptr) *key = nullptr;
    // The native part first (mode 7, what the shipped weapons use), then the rebuilt copy's slide.
    API::UObject* c = s_np_slide.get();
    API::UObject* src = s_np_src.get();
    const wchar_t* bone = s_np_bone_w.empty() ? nullptr : s_np_bone_w.c_str();
    if (c == nullptr) {
        for (int i = 0; i < s_sp_count; ++i)
            if (s_sp_parts[i].is_slide && !s_sp_parts[i].dropped) { c = s_sp_parts[i].comp.get(); break; }
        src = nullptr; bone = nullptr;
    }
    if (c == nullptr) return false;
    if (key != nullptr) *key = (void*)c;
    Vec3 v{};
    if (g_cfg.slide_zone == 2 && src != nullptr && bone != nullptr && call_socket_location(src, bone, &v)) { *out = v; return true; }
    if (g_cfg.slide_zone == 3 && call_ret_vec3(c, L"K2_GetComponentLocation", &v)) { *out = v; return true; }
    const char* how = "";
    if (part_world_centre(c, &v, &how)) { *out = v; return true; }
    return call_ret_vec3(c, L"K2_GetComponentLocation", out);
}

// ---- TAKE THE SNAPSHOT. One instant, one sequence number, before the reload state machine and
// every tick that reads it. Four pose reads become these four and no others; the camera is fetched
// once and CARRIED rather than re-fetched per transform; and the two game-side positions the tests
// compare against (the rack part, the magazine component) are read HERE, beside the poses, instead
// of a tick earlier or later in the path.
//
// Each game-side read is gated on its own consumer being able to want it, so an idle tick pays for
// nothing: the part only while the rack could run, the well only while a magazine is actually out.
void zone_snapshot_take() {
    ZoneSnap s{};
    s.seq = s_snap.seq + 1;
    if (!zone_snapshot_on()) { s_snap = s; return; }

    const auto hidx = API::VR::get_hmd_index();
    const auto aidx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                          : API::VR::get_right_controller_index();
    const auto oidx = g_cfg.aim_left_hand ? API::VR::get_right_controller_index()
                                          : API::VR::get_left_controller_index();
    Quat hq{};
    const bool head_ok = hidx >= 0 && get_pose(hidx, &s.head, &hq, /*use_aim=*/false);
    const bool aim_ok  = aidx >= 0 && get_pose(aidx, &s.aim, &s.aim_q, /*use_aim=*/true);
    const bool off_ok  = oidx >= 0 && get_pose(oidx, &s.off, &s.off_q, /*use_aim=*/false);
    s.poses_ok = head_ok && aim_ok && off_ok;
    if (aim_ok) {
        s.aim_fix = placement_aim_fix(s.aim_q);
        s.f = quat_forward(s.aim_fix);
        s.u = quat_rotate(s.aim_fix, Vec3{0.0f, 1.0f, 0.0f});
        s.r = quat_rotate(s.aim_fix, Vec3{1.0f, 0.0f, 0.0f});
    }
    // THE DRAWN WEAPON'S OWN FRAME, for mode 2. NOT g_p_wrot_* on its own: the palette publishes
    // the grip rotation and the weapon trim as DELTAS on the hand, not as an absolute pose, and
    // reading either as a frame would be a guess. This is the composition aimbore already uses to
    // find the drawn barrel (features/aimbore/AimBore.cpp): the aim-fixed hand pose, then the grip
    // rotation, then the roll trim, then the weapon trim, with the same sign and axis remap on
    // each published quaternion. palette_pose_trim_rotations answers only while the placement
    // actually owns the pose, which is exactly when mode 2 has anything to stand on.
    if (aim_ok) {
        float gq[4] = {0, 0, 0, 1}, wq[4] = {0, 0, 0, 1};
        if (palette_pose_trim_rotations(gq, wq)) {
            auto qn = [](Quat q) {
                const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
                return (n > 1.0e-6f && std::isfinite(n)) ? Quat{q.x / n, q.y / n, q.z / n, q.w / n}
                                                         : Quat{0.0f, 0.0f, 0.0f, 1.0f};
            };
            const Quat gx = qn(Quat{gq[1], gq[2], -gq[0], -gq[3]});
            const Quat wx = qn(Quat{wq[1], wq[2], -wq[0], -wq[3]});
            const float rh = palette_pose_roll_trim_deg() * 0.5f * DEG2RAD;
            const Quat rx = qn(Quat{0.0f, 0.0f, -std::sin(rh), -std::cos(rh)});
            const Quat q = quat_mul(quat_mul(quat_mul(s.aim_fix, gx), rx), wx);
            const float m2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
            if (m2 > 0.9f && m2 < 1.1f) { s.wpn_q = q; s.wpn_ok = true; }
        }
    }
    s.cam_ok = gesture_game_cam(&s.cam);
    s.cam_travel = s_gc_travel;   // gesture_game_cam integrated it for this same fetch
    if (reload_rack_available() && slide_weapon_ok()) s.part_ok = sl_part_world_now(&s.part, &s.part_key);
    if (s_reload != ReloadState::Idle) s.well_ok = mag_well_world_now(&s.well, &s.well_rot, &s.well_rot_ok);
    s_snap = s;
}
