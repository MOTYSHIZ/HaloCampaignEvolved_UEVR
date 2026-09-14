// palettewpn (fork feature, Experimental): the on-foot reticule: aim point, hosted widget and compositor publish, runnable without rig writes.
// Textual fragment, included by Plugin.cpp in its anonymous namespace, before update(). Moved verbatim; not compiled on its own.
static void onfoot_reticule_tick(API::UObject* rig, const Vec3& comp_world, double aim_yaw,
                                 double aim_pitch, uint32_t tick) {
    if (g_cfg.aim_reticule) {
        Vec3 origin{};
        const bool have_origin = (g_rig_parent != nullptr)
            && call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &origin);
        // Tell XrLayer WHICH gate stopped the publish, so its "layer DARK"
        // line can name the cause instead of reporting that nothing arrived.
        // g_rig_parent is the ARM RIG's attach parent, so this is the point at
        // which the compositor reticule depends on the arm lane resolving.
        if (!have_origin) xrlayer_note_publish_gate(3);

        // ANCHOR THE RAY WHERE THE PLAYER'S EYE IS, once the eye has left the
        // camera. The marker's job is to sit over the thing the shot will hit,
        // as SEEN BY THE PLAYER, and that only survives being drawn short if the
        // ray it sits on starts at the eye: every point on an eye-anchored ray
        // projects onto the same background point, so the visibility cap costs
        // nothing. Drawn short on a CAMERA-anchored ray it lies by
        // offset * (1/drawn - 1/actual) -- at 1 m of divergence, a 6 m cap and a
        // 30 m target, about 7 degrees. That is the "shots land at the reticule
        // instead of at what I'm pointing at" report, and it is why the marker
        // and the aim have to be built from ONE piece of geometry.
        //
        // Leashed, delta is ~0 and this is the same point as before.
        bool conv = halo::aim_converge_engaged();
        if (have_origin && conv) {
            Vec3 eye_d{};
            if (halo::aim_converge_delta(&eye_d)) {
                origin.x += eye_d.x; origin.y += eye_d.y; origin.z += eye_d.z;
            } else {
                conv = false;
            }
        }

        if (have_origin) {
            // Only borrow a prop if the cube path is actually wanted -- otherwise
            // the map quietly loses a piece of set dressing for nothing.
            if (g_cfg.aim_reticule_cube && !g_aim_marker.active) {
                g_aim_marker.active = resolve_marker(g_aim_marker, g_cfg.aim_reticule_cm,
                                                      comp_world, "aim reticule",
                                                      g_pivot_marker.actor);
                if (!g_aim_marker.active) g_cfg.aim_reticule_cube = false;
            } else if (!g_cfg.aim_reticule_cube && g_aim_marker.active) {
                release_marker(g_aim_marker, "aim reticule");
            }
            // NOT gated on g_aim_marker.active (the BORROWED LEVEL PROP): with
            // aimreticulecube=0 the marker is never acquired, so gating the
            // ray, the Lua publish or the world-space draw on it silently
            // disables them all -- the same trap as gating on piv_cube above.
            // Only park_marker belongs under that condition.
            {
                // UE forward from the reticule's chosen angles -- game aim, or
                // the smoothed controller setpoint. See reticule_ray_angles.
                //
                // ENGAGED, THE RAY IS THE INTENT, NOT THE ACHIEVED AIM. Those
                // are different rays once convergence is on: the achieved aim
                // is the intent BENT so a shot from the camera lands on the
                // target, so following it from an eye-anchored origin would
                // apply the correction twice and mark the wrong point. The
                // intent ray from the eye is the one whose hit both the marker
                // and the aim are derived from -- one piece of geometry, two
                // consumers.
                //
                // Passing it in as the "aim" argument also keeps the divergence
                // guard honest: with src=1 it would otherwise measure the
                // correction itself as loop error and snap every frame.
                double ray_yaw = aim_yaw, ray_pitch = aim_pitch;
                if (conv) {
                    ray_yaw   = (double)halo::g_desired_yaw.load();
                    ray_pitch = (double)halo::g_desired_pitch.load();
                }
                // RETFRESH (fork, aimreticulefresh, experimental): take the ray from ControlRotation read
                // back at this moment instead of the tick's early copy. Off by default.
                if (g_cfg.aim_reticule_fresh != 0) {
                    double fr_p = 0.0, fr_y = 0.0;
                    if (read_control_rotation_hook(&fr_p, &fr_y)) {
                        static uint32_t s_fresh_log = 0;
                        if (tick - s_fresh_log >= 64) {
                            s_fresh_log = tick;
                            API::get()->log_info(
                                "[Halo-CampE-UEVR] RETFRESH early=(y%.3f,p%.3f) "
                                "fresh=(y%.3f,p%.3f) d=(y%.3f,p%.3f) deg",
                                aim_yaw, aim_pitch, fr_y, fr_p,
                                wrap180((float)(fr_y - aim_yaw)),
                                (float)(fr_p - aim_pitch));
                        }
                        ray_yaw = fr_y; ray_pitch = fr_p;
                    }
                }
                float r_yaw = 0.0f, r_pitch = 0.0f;
                reticule_ray_angles(ray_yaw, ray_pitch, &r_yaw, &r_pitch);
                const float cp = std::cos(r_pitch * DEG2RAD);
                const Vec3 fwd{cp * std::cos(r_yaw * DEG2RAD),
                               cp * std::sin(r_yaw * DEG2RAD),
                               std::sin(r_pitch * DEG2RAD)};
                // TRACE FIRST, fixed distance as the fallback. On a hit the
                // marker lands on the surface, so it reads correctly from any
                // eye position -- which is the whole point, because the eye
                // moves with the player's head and the shot origin does not.
                // A failed resolve keeps the previous behaviour exactly.
                //
                // Only the DISTANCE is decided here; the target point is built
                // from it below. The traced hit is on this same ray by
                // construction, so deriving the point from d rather than using
                // the hit directly keeps the drawn position and the distance
                // that feeds the size compensation in agreement -- they must
                // not be able to disagree.
                float d = g_cfg.aim_reticule_dist;
                // THE SCOPE'S CONVERGENCE DISTANCE, kept SEPARATE from the
                // drawn one. `d` carries the 6 m visibility cap and the surface
                // standoff -- both display choices, made so the reticule sits at
                // a comfortable eye-convergence depth and does not pop between
                // depths when you pan onto sky. For a marker drawn along the ray
                // FROM THE EYE that is harmless: any point on the ray gives the
                // right DIRECTION, which is all a screen-space overlay needs.
                //
                // The scope capture is not on that ray. With scopecamorigin=1 it
                // sits on the WEAPON, so converging it on a 5-6 m point swings it
                // by the head-to-weapon parallax -- atan(40cm/500cm) is about
                // 4.6 degrees against a scope FOV of 4.38, i.e. MORE THAN THE
                // WHOLE FIELD OF VIEW. Measured 2026-09-07: RET PROJECT reported
                // z pinned at ~495 cm on every sample while the aim swept the
                // room, with the capture readback at err=0.00deg and the reticule
                // dead centre -- everything internally consistent, converging on
                // the wrong point.
                //
                // The aim lane already draws this exact distinction a few lines
                // below ("`h` and not `d`: the drawn distance carries the
                // visibility cap ... neither of which has anything to do with how
                // far the target actually is"). The capture is the second
                // consumer that needs the real range, and it was handed the
                // display one.
                float focus_d = g_cfg.aim_reticule_trace_max;
                // PER-LANE COOLDOWN, not a session kill. The first version
                // of this disabled the trace for the whole session after 3
                // faults, which pinned the reticule at aimreticuledist for
                // the rest of play -- trading a five-second transient for a
                // permanent regression. The fault window around a level
                // transition is ~5 s, so backing off and RETRYING is the
                // right shape; the backoff grows if it keeps failing, so a
                // genuinely broken lane still ends up parked on its own.
                if (tick < g_lane_retry_at[PERF_TRACE].load(std::memory_order_relaxed)) {
                    static uint32_t s_said_at = 0;
                    if (tick - s_said_at > 320) {
                        s_said_at = tick;
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] reticule: trace lane COOLING "
                            "DOWN after %u fault(s) -- fixed distance "
                            "(aimreticuledist=%.0f) until tick %u, then it "
                            "retries. Everything else keeps running.",
                            g_lane_faults[PERF_TRACE].load(std::memory_order_relaxed),
                            g_cfg.aim_reticule_dist,
                            g_lane_retry_at[PERF_TRACE].load(std::memory_order_relaxed));
                    }
                } else if (g_cfg.aim_reticule_trace) {
                    const float cap  = g_cfg.aim_reticule_max_dist;
                    const float tmax = g_cfg.aim_reticule_trace_max;
                    const Vec3 far_end{origin.x + fwd.x * tmax,
                                       origin.y + fwd.y * tmax,
                                       origin.z + fwd.z * tmax};
                    // Never let the trace land on the player. The pawn is one
                    // actor and the weapon is another, attached to the rig --
                    // ignoring only the pawn leaves the gun to catch the ray
                    // every time an animation swings it across the camera.
                    API::UObject* ignore[2] = {nullptr, nullptr};
                    int nignore = 0;
                    if (auto* pw = API::get()->get_local_pawn(0)) {
                        ignore[nignore++] = reinterpret_cast<API::UObject*>(pw);
                    }
                    if (auto* wa = fp_weapon_actor()) ignore[nignore++] = wa;

                    Vec3 hit{};
                    bool got = false;
                    HALO_VR_DEV_ONLY(hit_trace_dev_report());
                    {
                        // Inside the gate, so `n` counts real traces and `mean`
                        // is the cost of one -- not an average diluted by the
                        // frames that never traced.
                        PerfScope _pt(PERF_TRACE);
                        got = hit_trace(origin, far_end, ignore, nignore, &hit);
                    }
                    float h = 0.0f;
                    if (got) {
                        const float hx = hit.x - origin.x;
                        const float hy = hit.y - origin.y;
                        const float hz = hit.z - origin.z;
                        h = std::sqrt(hx * hx + hy * hy + hz * hz);
                        if (h > cap) {
                            // Beyond the cap: park at the cap EXACTLY. No
                            // surface offset -- there is no surface here to
                            // clip into, and subtracting one would just pull
                            // the marker off the depth we chose.
                            d = cap;
                        } else {
                            // Sit just in front of the wall rather than in it.
                            // Floored so a muzzle-contact hit cannot put the
                            // marker behind the eye.
                            d = h - g_cfg.aim_reticule_surface_off;
                            if (d < 20.0f) d = 20.0f;
                        }
                    } else {
                        // Miss (sky, or past the trace length). The cap, not
                        // aim_reticule_dist: panning off a wall onto sky should
                        // not pop the reticule between two depths.
                        d = cap;
                    }
                    // The capture converges on the REAL hit, and on a MISS it
                    // converges far rather than at the cap. Far is the safe
                    // failure here: at the trace limit the capture's line is
                    // effectively parallel to the shot line, so the parallax
                    // error goes to zero. The cap would reintroduce exactly the
                    // 4.6-degree swing this fixes.
                    focus_d = got ? h : tmax;

                    // THE RANGE, handed to the aim. `h` and not `d`: the drawn
                    // distance carries the visibility cap and the surface
                    // standoff, neither of which has anything to do with how
                    // far the target actually is. Aiming at the capped point
                    // would put the shot short of the wall the marker is
                    // painted on -- the exact confusion this feature exists to
                    // remove. Fed on a MISS too (as "no measurement"), because
                    // the filter's own miss policy is to hold, not to guess.
                    halo::aim_converge_feed(h, got, g_last_dt);

#if HALO_VR_DEV
                    // AIMCONV -- the whole correction on one line, so "my shots
                    // land left of the reticule" can be READ. offset is how far
                    // the eye has left the shot origin, range is what the trace
                    // measured, and dyaw/dpitch is the bend those two produce.
                    // If dyaw is 0 while offset is large, the feature is not
                    // engaged and the reason is one of the gates in
                    // aim_converge_engaged().
                    if (g_cfg.aim_conv_log > 0
                        && (tick % (uint32_t)g_cfg.aim_conv_log) == 0) {
                        Vec3 dl{};
                        const bool have_d = halo::aim_converge_delta(&dl);
                        float cy = halo::g_desired_yaw.load();
                        float cp2 = halo::g_desired_pitch.load();
                        const float iy = cy, ip = cp2;
                        const bool bent = halo::aim_converge_apply(&cy, &cp2);
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] AIMCONV eng=%d offset=(%.1f,%.1f,%.1f)cm "
                            "|off|=%.1f have=%d | hit=%d h=%.0f drawn=%.0f range=%.0fcm "
                            "| intent=(y%.2f,p%.2f) cmd=(y%.2f,p%.2f) d=(y%.2f,p%.2f) bent=%d",
                            (int)conv, dl.x, dl.y, dl.z,
                            std::sqrt(dl.x*dl.x + dl.y*dl.y + dl.z*dl.z), (int)have_d,
                            (int)got, h, d, halo::aim_converge_range(),
                            iy, ip, cy, cp2, wrap180(cy - iy), cp2 - ip, (int)bent);
                    }
#endif
                }
                const Vec3 target{origin.x + fwd.x * d,
                                  origin.y + fwd.y * d,
                                  origin.z + fwd.z * d};

                // ---- PUBLISH THE RAY TO LUA (the real reticule).
                //
                // Borrowing a level prop is only a diagnostic: it steals
                // set dressing, inherits whatever mesh happens to be nearby,
                // and cannot ship. uevrlib's reticule module builds its OWN
                // sphere mesh, so the Lua side owns the visual and this side
                // just supplies geometry.
                //
                // plugin -> Lua is the direction that actually exists in the
                // API (dispatch_lua_event); there is no Lua -> plugin channel,
                // which is why the split is this way round and not the reverse.
                if (g_cfg.aim_reticule_lua) {
                    char json[256];
                    snprintf(json, sizeof(json),
                             "{\"ox\":%.1f,\"oy\":%.1f,\"oz\":%.1f,"
                             "\"tx\":%.1f,\"ty\":%.1f,\"tz\":%.1f,\"dist\":%.1f}",
                             origin.x, origin.y, origin.z,
                             target.x, target.y, target.z, d);
                    API::get()->dispatch_lua_event("halo_aim_ray", json);


                    // Count it, and say so periodically. This site sits five
                    // conditions deep (rig_loc -> the piv/reticule group ->
                    // a successful component read -> aim_reticule -> a valid
                    // origin), so without the counter "no reticule appeared"
                    // could equally mean the plugin never published or the
                    // script never drew. This makes the two distinguishable.
                    static uint32_t dispatches = 0;
                    static uint32_t last_rep = 0;
                    ++dispatches;
                    if (tick - last_rep >= 600) {
                        last_rep = tick;
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] aim ray dispatched to Lua: %u so far, latest %s",
                            dispatches, json);
                    }
                }

                // THE RETICULE. World space, so it has no HUD bounds, and its
                // own geometry, so nothing is taken from the level -- the two
                // constraints the borrowed-prop marker cannot meet.
                // The game's own crosshair, in world space -- brings the
                // per-weapon art and the hit marker with it.
                // SCALE WITH DISTANCE. This used to be pinned to 1.0, which was
                // right only while the placement distance was a constant: the
                // scales were tuned at that one distance, so no compensation
                // was needed. Tracing removed the constant -- the reticule now
                // lands wherever the world is -- and at a fixed world size
                // apparent size goes as 1/distance, so it is overwhelming
                // against a near wall and invisible across a room.
                //
                // Proportional scaling cancels that. The floor stops the ring
                // vanishing when the muzzle is against a surface. See
                // aim_reticule_min_scale for how the two knobs set the anchor.
                // Anchored at BOTH ends -- see aim_reticule_min_scale. The far
                // anchor is the size this used to be pinned at, so the ring at
                // full distance is unchanged from before tracing existed and
                // only the near field is new.
                {
                    const float ms = g_cfg.aim_reticule_min_scale;
                    const float xs = g_cfg.aim_reticule_max_scale;
                    const float md = g_cfg.aim_reticule_min_scale_dist;
                    const float xd = g_cfg.aim_reticule_max_dist;
                    float s = xs;
                    if (xd > md) {
                        float t = (d - md) / (xd - md);
                        if (t < 0.0f) t = 0.0f;
                        if (t > 1.0f) t = 1.0f;   // d is capped already; belt and braces
                        s = ms + (xs - ms) * t;
                    }
                    g_ret_scale_mul = s;
                }

                if (g_cfg.aim_widget) {
                    // rescan gate: the pick has a consumer this tick
                    g_ret_ensure_seen_tick = g_ticks.load(std::memory_order_relaxed);
                    reticule_widget_ensure(rig);
                    reticule_widget_move(target, origin);
                } else {
                    // Turning the feature off must give the crosshair back --
                    // hosting it removed it from the HUD. Self-latching inside,
                    // so the steady off state costs one bool test.
                    reticule_widget_release();
                }

                // Plain sphere. Kept as the fallback that is known to render.
                // NOT gated on aim_mesh: the hide path lives inside
                // reticule_mesh_move, so gating the call would leave the quad
                // frozen and visible at its last position whenever the
                // feature is switched off.
                g_ret_origin = origin;
                g_have_ret_origin = true;

                // Publish the same target to the compositor reticule. It draws
                // ALONGSIDE the two above, not instead of them, and it needs the
                // CAMERA pose rather than the ray origin: it works by expressing
                // the reticule as an offset in the camera's own frame, which is
                // frame-invariant because the UE camera and the HMD are the same
                // object. Skipped without a composed view -- there is no camera
                // frame to be relative to yet.
                // The compositor reticule gets the TARGET only. Where that
                // point lands in stage space is decided at RENDER rate in
                // on_post_calculate_stereo_view_offset -- exactly like the
                // navpoint markers above, and for exactly the same reason.
                // RETSTAMP (fork): the latest trace depth and when it was taken, for the render-time publish.
                reticule_depth_note(d);
                if (g_cfg.stomp_log != 0) {
                    halo::stomp_mark(40, origin.x, origin.y, origin.z, d);
                    halo::stomp_mark(41, target.x, target.y, target.z, (float)features_reticule_render_publish_mode());
                    halo::stomp_mark(42, r_yaw, r_pitch, (float)ray_yaw, (float)ray_pitch);
                    halo::stomp_mark(43, g_view_pos_x.load(), g_view_pos_y.load(), g_view_pos_z.load(), (float)tick);
                }
                // Stamped reticle placement (aimreticulestamp 1/2) publishes at render instead, and only while
                // the palette weapon (armdriver mode 3) owns the aim. Otherwise the author's tick publish.
                if (features_reticule_render_publish_mode() == 0) {
                    xrlayer_note_publish_gate(0);   // reached the publish
                    xrlayer_notice_reticule(layer_anchor(halo::XRLAYER_SLOT_RETICULE, target),
                                            g_ret_scale_mul.load());
                }
                // Hand the scope THIS tick's aim ray, built RAW from the
                // game's live aim angles rather than from the reticule's
                // target: reticule_ray_angles layers display smoothing tuned
                // for a floating dot, and the first headset pass showed that
                // smoothing as the whole pane trailing the hand. Only the
                // direction matters; 500 cm is an arbitrary ray length.
                {
                    const float cp_s = std::cos((float)aim_pitch * DEG2RAD);
                    const Vec3 scope_target{
                        origin.x + cp_s * std::cos((float)aim_yaw * DEG2RAD) * 500.0f,
                        origin.y + cp_s * std::sin((float)aim_yaw * DEG2RAD) * 500.0f,
                        origin.z + std::sin((float)aim_pitch * DEG2RAD) * 500.0f};
                    scope_notice_ray(origin, scope_target, rig, tick);
                    // The REAL traced hit, for the capture's convergence only.
                    // scope_target above is an arbitrary 500 cm point and is
                    // right for DIRECTION; converging on it aims the scope past
                    // anything nearer, which shows up as the target sliding out
                    // of the pane when you strafe. `target` is the same point
                    // the reticule uses, which is why the reticule stayed on it
                    // while the image did not.
                    // NOT `target`: that is the DRAWN point, capped at 6 m for
                    // display comfort. The capture needs the real range -- see
                    // focus_d above.
                    {
                        const Vec3 focus_pt{origin.x + fwd.x * focus_d,
                                            origin.y + fwd.y * focus_d,
                                            origin.z + fwd.z * focus_d};
                        scope_notice_focus(focus_pt, true, tick);
                    }
                    // The SAME ray to XrLayer, from the SAME site, so the two
                    // cannot drift apart. It projects the reticule's (smoothed)
                    // target through this (raw) capture axis for
                    // xrlayerscopereticle=2 -- see xrlayer_note_scope_ray.
                    xrlayer_note_scope_ray(origin, scope_target);
                }
                if (g_cfg.aim_mesh) reticule_mesh_ensure(rig);

                // Park on the view axis for measurement runs (see aim_park_view).
                Vec3 mesh_target = target;
                if (g_cfg.aim_park_view && g_have_render_yaw.load()) {
                    const float vy = g_render_view_yaw.load();
                    const float vp = g_render_view_pitch.load();
                    const float cv = std::cos(vp * DEG2RAD);
                    mesh_target = Vec3{origin.x + cv * std::cos(vy * DEG2RAD) * d,
                                       origin.y + cv * std::sin(vy * DEG2RAD) * d,
                                       origin.z + std::sin(vp * DEG2RAD) * d};
                }
                reticule_mesh_move(mesh_target);

                if (g_cfg.aim_draw) {
                    draw_debug_sphere(rig, target, g_cfg.aim_draw_r,
                                      g_cfg.aim_draw_cr, g_cfg.aim_draw_cg,
                                      g_cfg.aim_draw_cb,
                                      g_cfg.aim_draw_dur, g_cfg.aim_draw_seg,
                                      g_cfg.aim_draw_th);

                    static uint32_t last_draw_rep = 0;
                    if (tick - last_draw_rep >= 600) {
                        last_draw_rep = tick;
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] reticule drawn at (%.0f,%.0f,%.0f) r=%.1f dur=%.3f",
                            target.x, target.y, target.z,
                            g_cfg.aim_draw_r, g_cfg.aim_draw_dur);
                    }
                }

                if (g_cfg.aim_reticule_cube) park_marker(g_aim_marker, target);

                // Read back where it actually landed. On this title "the call
                // succeeded" does not mean "the object moved", so the only
                // trustworthy check is the object's own reported location.
                if (tick % 600 == 0 && marker_alive(g_aim_marker)) {
                    Vec3 actual{};
                    if (call_ret_vec3(g_aim_marker.actor, L"K2_GetActorLocation", &actual)) {
                        const float ex = actual.x + g_aim_marker.center_off.x - target.x;
                        const float ey = actual.y + g_aim_marker.center_off.y - target.y;
                        const float ez = actual.z + g_aim_marker.center_off.z - target.z;
                        API::get()->log_info(
                            "[Halo-CampE-UEVR]   reticule target=(%.0f,%.0f,%.0f) err=%.1f cm  aim=(%.1f,%.1f)",
                            target.x, target.y, target.z,
                            std::sqrt(ex * ex + ey * ey + ez * ez),
                            (float)aim_pitch, (float)aim_yaw);
                    }
                }
            }
        }
    } else if (g_aim_marker.active) {
        release_marker(g_aim_marker, "aim reticule");
    }
}
