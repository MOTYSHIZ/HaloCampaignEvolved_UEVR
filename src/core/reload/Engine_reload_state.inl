// core reload engine (runs while reloadvr or slidevr is on): the per-weapon reload state (reloadstate) and reload_release_all().
// Textual fragment, included by core/reload/ReloadEngine.cpp at namespace halo scope, after the hidden display. Moved verbatim; not compiled on its own.
// ---- RELOAD STATE PER WEAPON (reloadstate, Config.hpp) -----------------------------------------
// The old memory had four holes, and each one hands a weapon back "mag in, racked":
//  1. it saved only when the weapon changed WHILE a reload was in flight, so a seated magazine
//     waiting for the rack, a locked-back slide or the phantom round followed the player;
//  2. gesture_reset (stick mode: vehicles, cutscenes, death; calibration) forced Idle and cleared
//     the flags without saving anything;
//  3. slide_phantom_tick dropped the phantom flag on the key change, on the very tick the
//     restore had put it back;
//  4. the magazine hide landed once on one component, and a swap spawns new components.
// Here the state is kept per weapon identity, saved on every exit path (and mirrored every tick
// with reload_state_save 1), and loaded whenever that identity is in hand again.
struct RsRec {
    std::string id;            // "i:0x<datum>" (instance) or "t:<type>" (type name); empty = free slot
    std::string type;
    int32_t     datum = -1;
    uint32_t    epoch = 0;
    long long   touched = 0;
    ReloadState state = ReloadState::Idle;
    float       grab_y = 0.0f;
    int         chamber_left = 0;
    int32_t     lock_frame = 0;
    int         coop_lock_rounds = 1;
    bool        empty_at_drop = false, locked_back = false, lock_pending = false, reload_due = false;
    bool        pressed_early = false, press_pending = false, true_empty = false, hide_display = false;
};
constexpr int RS_MAX = 16;
RsRec         s_rs_recs[RS_MAX];
std::string   s_rs_live_id;              // identity the live state belongs to; empty = not synced
std::string   s_rs_live_type;            // type of the weapon the live state belongs to (also while its id is pending)
int32_t       s_rs_live_datum = -1;
std::string   s_rs_seen_type;
API::UObject* s_rs_seen_actor = nullptr; // identity only, never dereferenced
long long     s_rs_since = 0;            // when the type or actor in hand last changed (the datum wait)
API::UObject* s_rs_pc = nullptr;         // identity only
uint32_t      s_rs_epoch = 1;
std::string   s_rs_recent[2];            // the last two identities held
std::string   s_rs_carry[2];             // reloadstate 3: those two, frozen at the last level change
long long     s_rs_carry_until = 0;
bool          s_rs_death_cleared = false;
std::string   s_rs_hands[2];             // the two types carried, the one in hand first
bool          s_rs_after_reset = false;  // the next identity adoption follows a gesture reset
bool          s_rs_live_no_manual = false;   // the weapon in hand has no manual reload (reloadskipweapons)
int32_t       s_rs_loaded_datum = -1;    // per type: the object that wrote the record just loaded
bool          s_rs_loaded_after_reset = false;
bool          s_rs_loaded_old_epoch = false;

int32_t rs_read_datum(API::UObject* wpn) {
    if (wpn == nullptr) return -1;
    auto** pc = wpn->get_property_data<API::UObject*>(L"BlamObjectSynchronization");
    if (pc == nullptr || IsBadReadPtr(pc, sizeof(void*)) || *pc == nullptr || IsBadReadPtr(*pc, sizeof(void*))) return -1;
    auto* p = (*pc)->get_property_data<int32_t>(L"BlamObjectIndex");
    if (p == nullptr || IsBadReadPtr(p, sizeof(int32_t))) return -1;
    return *p;
}
void rs_capture(RsRec& r) {
    r.state = s_reload; r.grab_y = s_grab_y;
    r.chamber_left = s_sl_chamber_left; r.lock_frame = s_sl_lock_frame; r.coop_lock_rounds = s_coop_lock_rounds;
    r.empty_at_drop = s_sl_empty_at_drop; r.locked_back = s_sl_locked_back; r.lock_pending = s_sl_lock_pending;
    r.reload_due = s_sl_reload_due; r.pressed_early = s_sl_pressed_early;
    // A press scheduled 150 ms after the chambered shot belongs to THIS gun: carried as pending,
    // it goes out at the seat instead of into whichever weapon is in hand when the timer runs out.
    r.press_pending = s_sl_press_pending || s_sl_press_due_at != 0;
    r.true_empty = s_true_empty; r.hide_display = s_hide_display;
}
bool rs_clean(const RsRec& r) {
    return r.state == ReloadState::Idle && !r.locked_back && !r.lock_pending && !r.reload_due && !r.pressed_early
        && !r.press_pending && !r.true_empty && !r.hide_display;
}
bool rs_same(const RsRec& a, const RsRec& b) {
    return a.state == b.state && a.grab_y == b.grab_y && a.chamber_left == b.chamber_left && a.lock_frame == b.lock_frame
        && a.coop_lock_rounds == b.coop_lock_rounds && a.empty_at_drop == b.empty_at_drop && a.locked_back == b.locked_back
        && a.lock_pending == b.lock_pending && a.reload_due == b.reload_due && a.pressed_early == b.pressed_early
        && a.press_pending == b.press_pending && a.true_empty == b.true_empty && a.hide_display == b.hide_display;
}
void rs_log(const char* verb, const char* why, const RsRec& r) {
    if (!g_cfg.reload_state_log) return;
    API::get()->log_info("[Halo-CampE-UEVR] RSTATE %s (%s) id=%s type=%s datum=0x%08X epoch=%u | state=%s chamber=%d "
                         "empty_at_drop=%d locked_back=%d lock_pending=%d reload_due=%d pressed_early=%d press_pending=%d "
                         "true_empty=%d hide_display=%d lock_frame=%d coop_lock=%d grab_y=%.3f",
                         verb, why, r.id.c_str(), r.type.c_str(), (unsigned)r.datum, (unsigned)r.epoch, state_name(r.state),
                         r.chamber_left, (int)r.empty_at_drop, (int)r.locked_back, (int)r.lock_pending, (int)r.reload_due,
                         (int)r.pressed_early, (int)r.press_pending, (int)r.true_empty, (int)r.hide_display,
                         (int)r.lock_frame, r.coop_lock_rounds, r.grab_y);
}
RsRec* rs_find(const std::string& id) {
    if (id.empty()) return nullptr;
    for (auto& r : s_rs_recs) if (!r.id.empty() && r.id == id) return &r;
    return nullptr;
}
RsRec* rs_slot() {
    RsRec* oldest = &s_rs_recs[0];
    for (auto& r : s_rs_recs) {
        if (r.id.empty()) return &r;
        if (r.touched < oldest->touched) oldest = &r;
    }
    if (g_cfg.reload_state_log) API::get()->log_info("[Halo-CampE-UEVR] RSTATE table full: forgetting the oldest record %s", oldest->id.c_str());
    return oldest;
}
void rs_forget_all(const char* why) {
    for (auto& rec : s_rs_recs) rec = RsRec{};
    s_rs_hands[0].clear(); s_rs_hands[1].clear();
    s_rs_loaded_datum = -1;
    if (g_cfg.reload_state_log) API::get()->log_info("[Halo-CampE-UEVR] RSTATE forget every record (%s)", why);
}
void rs_forget_type(const std::string& type, const char* why) {
    for (auto& rec : s_rs_recs) if (!rec.id.empty() && rec.type == type) { rs_log("forget", why, rec); rec = RsRec{}; }
}
// Halo carries two weapons, and a pickup swaps out the one in the hand.
void rs_hands_note(const std::string& type) {
    if (type.empty() || type == s_rs_hands[0]) return;
    if (type == s_rs_hands[1]) { std::swap(s_rs_hands[0], s_rs_hands[1]); return; }
    if (g_cfg.reload_state_id == 1 && g_cfg.reload_state_drop == 1 && !s_rs_hands[0].empty() && !s_rs_hands[1].empty()) {
        const std::string dropped = s_rs_hands[0];
        if (g_cfg.reload_state_log)
            API::get()->log_info("[Halo-CampE-UEVR] RSTATE carried [%s] [%s], %s came in: %s was swapped for it", s_rs_hands[0].c_str(), s_rs_hands[1].c_str(), type.c_str(), dropped.c_str());
        rs_forget_type(dropped, "swapped for a weapon on the ground");
        s_rs_hands[0] = type;
        return;
    }
    s_rs_hands[1] = s_rs_hands[0]; s_rs_hands[0] = type;
}
// Per type: the record for this type was written by a different Blam object than the one in hand.
// True when the record was forgotten.
bool rs_other_object(const std::string& type, int32_t saved, int32_t held, bool after_reset, bool old_epoch, const char* when) {
    const bool death = after_reset && g_cfg.reload_state_death == 2;
    const bool drop  = !after_reset && !old_epoch && g_cfg.reload_state_drop == 2;
    if (g_cfg.reload_state_log)
        API::get()->log_info("[Halo-CampE-UEVR] RSTATE %s: the %s record was saved by object 0x%08X, the one in hand is 0x%08X (after reset %d, older level %d) -> %s",
                             when, type.c_str(), (unsigned)saved, (unsigned)held, (int)after_reset, (int)old_epoch,
                             death ? "respawn, forget every record" : (drop ? "another weapon of this type, forget its record" : "record KEPT"));
    if (death) { rs_forget_all("respawn: the weapon in hand is a new object"); return true; }
    if (drop)  { rs_forget_type(type, "another weapon of this type, the record's own was left behind"); return true; }
    return false;
}
// The live state into its weapon's record. A clean state (nothing in progress) removes the record.
void rs_save(const char* why, bool quiet_if_same) {
    if (s_rs_live_id.empty()) return;
    RsRec now; now.id = s_rs_live_id; now.type = s_rs_live_type; now.datum = s_rs_live_datum; now.epoch = s_rs_epoch;
    rs_capture(now);
    RsRec* r = rs_find(s_rs_live_id);
    if (s_rs_live_no_manual) {   // no manual reload on this weapon: it never holds a record
        if (r != nullptr) { rs_log("forget", "weapon has no manual reload", *r); *r = RsRec{}; }
        return;
    }
    if (now.datum == -1 && r != nullptr) now.datum = r->datum;   // an unreadable tick never erases the saved object
    if (rs_clean(now)) {
        if (r != nullptr) { rs_log("clear", why, now); *r = RsRec{}; }
        else if (!quiet_if_same) rs_log("save", why, now);
        return;
    }
    const bool same = (r != nullptr) && rs_same(*r, now);
    if (r == nullptr) r = rs_slot();
    now.touched = now_ticks();
    *r = now;
    if (!(same && quiet_if_same)) rs_log("save", why, now);
}
// ---- THE FIVE OPEN WINDOWS. A press on its way to the game, the FirstPersonState hold, the
// animation rate clamp, the Wwise mute window and the first-person pose hold. None of them is a
// record and none is cleared by forgetting one: each is a timer already running against a weapon
// and a body, and both can go away underneath it. Extracted here so the two edges that end a
// body -- the gesture reset (which a death reaches through stick mode) and the level load -- close
// exactly the same five, rather than one of them depending on the other happening to fire.
void reload_release_windows(const char* why) {
    if (!g_cfg.reload_reset_holds) return;
    g_reload_hold_until.store(0, std::memory_order_relaxed);
    s_sl_press_at = 0;
    s_sh_until = 0; s_sh_inst = TrackedObject{};
    if (s_anim_rate_until != 0) s_anim_rate_until = 1;   // the next tick hands rate and pause back
    if (s_akm_until != 0) ak_id_mute_end();
    reload_pose_hold(0);
    // A tap made while the window held the tick belongs to the body that is gone: the first frame
    // control comes back must not fire it.
    s_reset_drop_tap = true;
    if (g_cfg.reload_vr_log || g_cfg.reload_state_log)
        API::get()->log_info("[Halo-CampE-UEVR] RELOAD released the press, the state hold, the rate clamp, the sound mute and the pose hold (%s)", why);
}
// The live state back to "nothing in progress" for the weapon now in hand.
void rs_live_fresh(const char* why) {
    if (s_reload != ReloadState::Idle) set_state(ReloadState::Idle, why);
    s_sl_chamber_left = 0; s_sl_empty_at_drop = false; s_sl_locked_back = false; s_sl_lock_pending = false;
    s_sl_reload_due = false; s_sl_pressed_early = false; s_sl_press_pending = false; s_sl_press_due_at = 0;
    s_sl_lock_frame = 0; s_sl_rack_done = false;
    s_true_empty = false; s_hide_display = false; s_coop_lock_rounds = 1;
    // A hand on the slide does not travel with the gun: the weapon left the hand.
    s_sl_held = false; s_sl_racked = false; s_sl_th = false; s_sl_release_at = 0;
    g_slide_pull.store(0.0f, std::memory_order_relaxed);
    s_reload_weapon.clear();
    s_ph_rebase = true;
}
void rs_live_load(const RsRec& r, const char* why) {
    s_sl_chamber_left = r.chamber_left; s_sl_empty_at_drop = r.empty_at_drop; s_sl_locked_back = r.locked_back;
    s_sl_lock_pending = r.lock_pending; s_sl_reload_due = r.reload_due; s_sl_pressed_early = r.pressed_early;
    s_sl_press_pending = r.press_pending; s_sl_press_due_at = 0; s_sl_lock_frame = r.lock_frame; s_sl_rack_done = false;
    s_true_empty = r.true_empty; s_hide_display = r.hide_display; s_coop_lock_rounds = r.coop_lock_rounds;
    s_grab_y = r.grab_y;
    if (r.state != ReloadState::Idle) {
        s_reload_weapon = s_rs_live_type;
        s_restoring_mag_out = true;   // no second magazine falls and no press goes out on a restore
        set_state(r.state, why);
        s_restoring_mag_out = false;
    }
    rs_log("restore", why, r);
}
void reload_state_track() {
    const long long nowt = now_ticks();
    // A replaced PlayerController is a map load (the aim reference keys off the same change).
    if (auto* pc = API::get()->get_player_controller(0)) {
        if (pc != s_rs_pc) {
            if (s_rs_pc != nullptr) {
                ++s_rs_epoch;
                // THE LIVE STATE GOES WITH THE BODY, WHATEVER reloadstatelevel SAYS ABOUT THE
                // RECORDS. Forgetting records is his key's business; the locks and the five open
                // windows belong to a weapon on a PlayerController that no longer exists, and
                // this edge used to leave every one of them running on the hope that stick mode
                // would fire a gesture reset on the same load. The state is saved to its record
                // first (so reloadstatelevel=1 still restores it), then the live side is cleared
                // exactly as the reset edge clears it.
                {
                    const char* why = "level change: the body and its weapon are gone";
                    if (!s_rs_live_id.empty() || !s_rs_live_type.empty()) {
                        rs_save(why, false);
                        s_rs_live_id.clear(); s_rs_live_type.clear(); s_rs_live_datum = -1;
                        s_rs_after_reset = true;
                    }
                    rs_live_fresh(why);
                    reload_release_windows(why);
                }
                if (g_cfg.reload_state_level == 0) rs_forget_all("level change, reloadstatelevel 0");
                s_rs_carry[0] = s_rs_recent[0]; s_rs_carry[1] = s_rs_recent[1];
                s_rs_carry_until = nowt + ms_to_ticks(120000);
                if (g_cfg.reload_state_log)
                    API::get()->log_info("[Halo-CampE-UEVR] RSTATE level change: epoch %u, carry candidates [%s] [%s]",
                                         (unsigned)s_rs_epoch, s_rs_carry[0].c_str(), s_rs_carry[1].c_str());
            }
            s_rs_pc = pc;
        }
    }
    auto* actor = fp_weapon_actor();
    const std::string type = weapon_key();
    const int32_t datum = rs_read_datum(actor);
    s_rs_cur_datum = datum;
    if (actor != s_rs_seen_actor || type != s_rs_seen_type) {
        if (g_cfg.reload_state_log && actor != nullptr)
            API::get()->log_info("[Halo-CampE-UEVR] RSTATE in hand: type=%s stem=%s datum=0x%08X (%s actor) mag=%d rack=%d chamber=%d always=%d nomanual=%d",
                                 type.c_str(), weapon_stem_lc_for(type).c_str(), (unsigned)datum, type == s_rs_seen_type ? "a NEW" : "another",
                                 (int)(native_mag_mesh_impl(false) != nullptr), (int)weapon_in_list(g_cfg.slide_weapons), (int)weapon_in_list(g_cfg.slide_chamber_weapons),
                                 (int)weapon_in_list(g_cfg.slide_always_weapons), (int)weapon_in_list(g_cfg.reload_skip_weapons));
        s_rs_seen_actor = actor; s_rs_seen_type = type; s_rs_since = nowt;
    }
    if (type.empty()) return;   // nothing resolved this tick: the live state stays with its weapon
    // A different TYPE is a different weapon now, even while its datum is unreadable: the previous
    // weapon's state must not ride onto it for a single tick.
    if (!s_rs_live_type.empty() && type != s_rs_live_type) {
        rs_save("weapon left the hand", false);
        rs_live_fresh("weapon changed, its state is kept with it");
        s_rs_live_id.clear(); s_rs_live_datum = -1;
    }
    rs_hands_note(type);
    s_rs_live_type = type;
    // An instance in hand whose datum blinks unreadable (same type) is held, not re-keyed: a
    // type-name stand-in here would look like another weapon and release the magazine for a tick.
    if (g_cfg.reload_state_id != 1 && datum == -1 && s_rs_live_id.rfind("i:", 0) == 0) return;
    std::string id;
    if (g_cfg.reload_state_id == 1) id = "t:" + type;
    else if (datum != -1) { char b[24]; sprintf_s(b, "i:0x%08X", (unsigned)datum); id = b; }
    else if (nowt - s_rs_since >= ms_to_ticks(g_cfg.reload_state_wait_ms)) id = "t:" + type;
    if (!id.empty() && id == s_rs_live_id) {
        // Per type: the object behind a loaded record is proved once the datum reads (a first tick
        // can be unreadable). Nothing is ever forgotten for an unreadable datum.
        if (s_rs_live_datum == -1 && datum != -1) {
            s_rs_live_datum = datum;
            if (g_cfg.reload_state_id == 1 && s_rs_loaded_datum != -1 && datum != s_rs_loaded_datum) {
                const int32_t saved = s_rs_loaded_datum;
                s_rs_loaded_datum = -1;
                if (rs_other_object(type, saved, datum, s_rs_loaded_after_reset, s_rs_loaded_old_epoch, "late datum"))
                    rs_live_fresh("the loaded record belonged to another object");
            }
        }
        return;
    }
    if (id.empty()) return;
    // The type-name stand-in gives way to the instance once its datum reads (same weapon, late).
    const bool late_datum = !s_rs_live_id.empty() && s_rs_live_id.rfind("t:", 0) == 0 && id.rfind("i:", 0) == 0;
    if (late_datum) {
        if (RsRec* old = rs_find(s_rs_live_id)) *old = RsRec{};
    } else if (!s_rs_live_id.empty()) {
        // Same type, another instance: a second rifle, a pickup.
        rs_save("weapon left the hand", false);
        rs_live_fresh("another weapon of the same type, its state is kept with it");
    }
    RsRec live; rs_capture(live);
    const bool live_dirty = !rs_clean(live);
    s_rs_live_id = id; s_rs_live_datum = datum;
    s_rs_live_no_manual = weapon_in_list(g_cfg.reload_skip_weapons);
    const bool after_reset = s_rs_after_reset; s_rs_after_reset = false;
    s_rs_loaded_datum = -1;
    if (s_rs_recent[0] != id) { s_rs_recent[1] = s_rs_recent[0]; s_rs_recent[0] = id; }
    for (auto& c : s_rs_carry) if (c == id) c.clear();   // matched directly: its datum survived the load
    RsRec* r = rs_find(id);
    if (r == nullptr && g_cfg.reload_state_id == 3 && nowt < s_rs_carry_until) {
        for (auto& c : s_rs_carry) {
            if (c.empty()) continue;
            RsRec* cr = rs_find(c);
            if (cr == nullptr || cr->type != type) continue;
            if (g_cfg.reload_state_log)
                API::get()->log_info("[Halo-CampE-UEVR] RSTATE level carry: %s adopts the record of %s (type %s)", id.c_str(), c.c_str(), type.c_str());
            cr->id = id; cr->datum = datum; r = cr; c.clear();
            break;
        }
    }
    if (r != nullptr && s_rs_live_no_manual) {
        // A weapon with no manual reload never takes a record, whatever wrote one.
        rs_log("forget", "weapon has no manual reload, a record never applies", *r);
        *r = RsRec{}; r = nullptr;
    }
    if (r != nullptr && g_cfg.reload_state_id == 1 && r->datum != -1 && datum != -1 && r->datum != datum) {
        if (rs_other_object(type, r->datum, datum, after_reset, r->epoch != s_rs_epoch, "in hand")) r = nullptr;
    }
    if (r != nullptr) { s_rs_loaded_datum = r->datum; s_rs_loaded_after_reset = after_reset; s_rs_loaded_old_epoch = (r->epoch != s_rs_epoch); }
    if (live_dirty && !s_rs_live_no_manual) {
        // The player acted while this identity was pending: that is newer than any record.
        rs_save("identity resolved, the live state is kept", false);
        return;
    }
    if (r != nullptr) { const RsRec copy = *r; rs_live_load(copy, "weapon in hand again"); }
    else if (g_cfg.reload_state_log)
        API::get()->log_info("[Halo-CampE-UEVR] RSTATE fresh (no record) id=%s type=%s datum=0x%08X epoch=%u",
                             id.c_str(), type.c_str(), (unsigned)datum, (unsigned)s_rs_epoch);
}
// gesture_reset forces Idle (stick mode, calibration, a gate): the state goes to its weapon first.
void reload_state_on_reset() {
    if (g_cfg.reload_state_id == 0) return;
    const bool dead = g_dbg_persp.load(std::memory_order_relaxed) == 2;
    if (!s_rs_live_id.empty() || !s_rs_live_type.empty()) {
        rs_save(dead ? "gesture reset, death camera" : "gesture reset (stick mode, calibration or gate)", false);
        rs_live_fresh("gesture reset, state kept with its weapon");
        s_rs_live_id.clear(); s_rs_live_type.clear(); s_rs_live_datum = -1;
        s_rs_after_reset = true;
    }
    if (dead && g_cfg.reload_state_death == 1 && !s_rs_death_cleared) {
        // A death drops the guns; the respawn loadout starts with nothing in progress.
        rs_forget_all("death camera");
        s_rs_death_cleared = true;
    }
    if (!dead) s_rs_death_cleared = false;
}
void reload_state_mirror() {
    if (g_cfg.reload_state_id == 0 || g_cfg.reload_state_save != 1) return;
    rs_save("mirror", true);
}
// MANUAL RELOAD SWITCHED OFF. Every lock the reload keeps (the phantom round, the dry stop, the hidden
// reload, the lock-back waiting for the rack) is ended only by the gesture, and with reloadvr off the
// gesture never runs: the gun stayed dead, and with the per-weapon records the flag no longer even
// dropped on a swap. The game's own reload runs unmanaged while it is off, so no record or legacy
// memory may survive to hand a gun back "mag out" when the switch comes on again.
void reload_release_all(const char* why) {
    const bool coop = g_cfg.coop_auto && net_is_coop();
    if (s_true_empty && !coop && !reload_hidden_mode()) {
        if (auto* r = rounds_field()) { if (*r == 1) *r = 0; }   // the phantom round back to the honest count
    }
    if (s_sl_lock_pending) {
        if (auto* animbp = reload_weapon_anim_instance())
            if (auto* p = animbp->get_property_data<int32_t>(L"PrimaryAmmunition_ExplicitFrame"))
                if (!IsBadWritePtr(p, sizeof(int32_t))) *p = slide_forward_frame();
    }
    if (s_reload != ReloadState::Idle) set_state(ReloadState::Idle, why);
    s_sl_chamber_left = 0; s_sl_empty_at_drop = false; s_sl_locked_back = false; s_sl_lock_pending = false;
    s_sl_reload_due = false; s_sl_pressed_early = false; s_sl_press_pending = false; s_sl_press_due_at = 0;
    s_sl_lock_frame = 0; s_sl_rack_done = false;
    s_true_empty = false; s_hide_display = false; s_coop_lock_rounds = 1;
    g_wristhud_hide_cradle.store(false, std::memory_order_relaxed);
    for (auto& m : s_wpn_mem) m = WpnMem{};
    rs_forget_all(why);
    s_rs_live_id.clear(); s_rs_live_type.clear(); s_rs_live_datum = -1; s_rs_cur_datum = -1;
    s_rs_seen_actor = nullptr; s_rs_seen_type.clear(); s_rs_after_reset = false;
    s_reload_weapon.clear();
    s_ph_rebase = true;
    if (g_cfg.reload_vr_log || g_cfg.reload_state_log) API::get()->log_info("[Halo-CampE-UEVR] RELOAD released every lock and record (%s)", why);
}
