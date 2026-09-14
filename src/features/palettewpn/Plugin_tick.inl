// palettewpn (fork feature, Experimental): the per-tick palette weapon work: arm hide, rendered-hand poses, parent frame, projection scale, judder and barrel instruments, mesh constants, axis probe.
// Textual fragment, included by Plugin.cpp inside update(), after the per-weapon deltas. Moved verbatim; not compiled on its own.
    // ---- PALETTE WEAPON MODE (armdriver 3, experimental). The fork's per-tick placement work, at
    // the fork's position in the tick: the FP arm hide (arms_update, which normally calls it, only
    // runs under UeRig), then the rendered-hand poses the sim-thread palette hook composes from.
    // Its two-hand hold runs after gesture_update in the tick body, as the fork ordered it (the
    // latch must see the gestures settled for this tick).
    if (palette_weapon_mode()) {
        g_tick_stage = "arms_hide";
        arms_hide_update();
        g_tick_stage = "palette_publish";
        blam_palette_publish_poses();
    }

    // The stock FP arms give way to the palette-driven weapon (doctrine in Arms.cpp).

    // ---- THE RENDERED-HAND POSES, EVERY FRAME, published for the palette hook (sim thread).
    // A pose published on the config cadence left the weapon transforming against a hand
    // position up to half a second stale -- the gun swung toward where you USED to point.
    // The two-hand hold: latch, blend ramp, haptics. After the pose publish on purpose -- both
    // read the same grip state, and this one must see it settled for this tick.
    features_game_tick_vehicle();

    // ---- THE PARENT FRAME, MEASURED RATHER THAN MODELLED -- published for the palette weapon.
    //
    // The FP mesh renders every palette node under its attach parent's ACTUAL rotation, and the
    // original 0.2's rig block (below) is explicit about why that value must be READ from the
    // component rather than inferred: ControlRotation, the locked view base, and the aim quat
    // were each tried as reconstructions of it during the palette effort, and each failed with a
    // different geometry of "the gun follows the camera". One reflected call per tick, the same
    // price the rig has always paid. Fail closed: valid=false publishes no divide, and the
    // palette publisher refuses to emit a pose in a frame it cannot name.
    // ---- DECOUPLED PITCH STAYS ON. This is the one UEVR invariant where we must NOT copy 0.5.
    //
    // 0.5 pins decoupled pitch OFF, and it is safe for them because their controller never touches
    // the camera (AimMethod::GAME). Ours drives the game camera FROM the controller by design --
    // so with decoupled pitch off, the aim pitch went straight into the rendered view: "my
    // controller is controlling the pitch", and it made the player sick within a minute. That was
    // this block, one revision ago, pinning it off on a watchdog. Reversed: decoupled pitch is
    // pinned ON, which is what the profile shipped with and what a hand-driven camera requires.
    // The palette frame math already carries the camera pitch explicitly (comp_rot = aim rotator,
    // pitch included), so it does not depend on the view being coupled.
    if (g_cfg.pin_uevr_frame && (tick % 64u) == 0u) {
        if (!API::VR::is_decoupled_pitch_enabled()) {
            API::VR::set_decoupled_pitch_enabled(true);
            API::get()->log_info("[Halo-CampE-UEVR] UEVRPIN: decoupled pitch was OFF -- restored ON "
                                 "(hand-driven aim must never pitch the rendered view)");
        }
    }

    // ---- THE FIRST-PERSON PROJECTION SCALE. What the reference disables before anything else.
    //
    // Halo's UE 5.5 camera enables the engine's first-person primitive scale with
    // FirstPersonScale = 0.15: the FP rig -- our palette's mesh -- is drawn shrunk toward the
    // camera and re-projected. The 0.5 mod ships a script (halo_first_person_projection_fix)
    // whose whole job is to set that to 1.0 and turn the flag off; their pose-matrix validation
    // (palette node within 4e-8 m of the 3.048-based prediction) is taken with it disabled. We
    // never disabled it: every palette placement this codebase has ever rendered went through a
    // 0.15 depth-dependent squash, which is a lever proportional to camera distance -- and the
    // socket readback that measured "456 cm per unit" measured the squash, not the palette.
    //
    // Same policy as their script: apply once per new camera object, and re-apply only if game
    // logic explicitly turns the scale back on. Not rewritten every tick. FOV override handled
    // the same way (their REMOVE_FIRST_PERSON_FOV = true). g_rig_parent IS the CameraComponent
    // -- the object their script finds by class -- so no second lookup.
    g_tick_stage = "fp_scale";
    if (g_cfg.fp_scale_fix && g_rig_parent != nullptr) {
        auto* cam = g_rig_parent;
        auto* enabled = cam->get_property_data<bool>(L"bEnableFirstPersonScale");
        auto* scale   = cam->get_property_data<float>(L"FirstPersonScale");
        if (enabled != nullptr && scale != nullptr) {
            const bool new_camera = (g_fpscale_camera != cam);
            if (new_camera || *enabled) {
                const float old_scale = *scale;
                const bool  old_enabled = *enabled;
                *scale = 1.0f;
                *enabled = false;
                API::get()->log_info("[Halo-CampE-UEVR] FPSCALE camera=%p FirstPersonScale %.3f "
                                     "enabled=%d -> 1.000/false%s",
                                     (void*)cam, old_scale, (int)old_enabled,
                                     new_camera ? " (new camera)" : " (game re-enabled it)");
            }
            auto* fov_enabled = cam->get_property_data<bool>(L"bEnableFirstPersonFieldOfView");
            auto* fov_fp      = cam->get_property_data<float>(L"FirstPersonFieldOfView");
            auto* fov_world   = cam->get_property_data<float>(L"FieldOfView");
            if (fov_enabled != nullptr && fov_fp != nullptr && fov_world != nullptr &&
                (new_camera || *fov_enabled)) {
                *fov_fp = *fov_world;
                *fov_enabled = false;
            }
            g_fpscale_camera = cam;
        } else {
            static bool s_said = false;
            if (!s_said) {
                s_said = true;
                API::get()->log_info("[Halo-CampE-UEVR] FPSCALE: CameraComponent has no "
                                     "FirstPersonScale property on this build -- fix inert");
            }
        }
    }

    // ---- THE WHOLE PICTURE, ONE LINE, ONE FRAME, EVERY SECOND.
    //
    // From the headset: "I don't understand why this is so hard. Add extensive logging -- camera,
    // controller, gun position and rotations for each, and the aim ray." He is right: every
    // diagnostic so far printed one term and reasoned about the others. This prints all of them
    // TOGETHER, in the same UE world frame, from the same tick, so a disagreement between any
    // two is visible by inspection instead of by derivation. Rows: HEAD (HMD, room-relative,
    // metres, UE axes) / HAND (grip pose, room-relative, metres, UE axes) / CAM (ControlRotation:
    // the Blam camera the palette embeds in) / VIEW (what the stereo callback actually rendered)
    // / GUN (the FP mesh's PrimaryWeapon socket, read back from the posed skeleton -- where the
    // gun REALLY is, in world cm, and its rotation) / AIM (the aim ray's origin and direction).
    // The barrel-axis MEASUREMENT (feeds the barrel lock) lives in here and must run whether or
    // not the lines are logged: palettewpnlog gates the log_info calls only.
    // EVERY TICK WHILE A FREEZE KEY IS HELD, every 45 otherwise. DIAGNOSTIC CADENCE ONLY.
    // A hold is 1-2 s; at 45 ticks that is two or three samples, which is not enough to
    // characterise motion the player can see -- and a reading taken at that rate was once
    // reported as "the gun is holding still" when it was moving several cm. Held, this gives
    // ~130 samples/second of the rendered socket. Costs nothing when the key is up.
    // JUDDERLOG: every tick, capped, and it turns the trace lines on by itself.
    static int s_jl_lines = 0;
    const bool jl = g_cfg.judder_log > 0 && s_jl_lines < g_cfg.judder_log;
    const uint32_t trace_every = (halo::blam_palette_freeze_active() || jl) ? 1u : 45u;
    const bool pwl = g_cfg.palette_weapon_log || jl;
    if (jl) ++s_jl_lines;
    // The barrel axis is only measured while the palette weapon owns (mode 3). Otherwise it must stop
    // CLAIMING validity, or aimbore=3 and the barrel lock would use an axis from a pose no longer drawn.
    if (!palette_weapon_mode()) halo::g_barrel_axis_valid.store(false, std::memory_order_relaxed);
    if (palette_weapon_mode() && (tick % trace_every) == 0u) {
        // Head and hand, room-relative, UE axes.
        Vec3 hpos{}, gpos{}; Quat hq{}, gq{};
        const auto hidx = API::VR::get_hmd_index();
        const auto cidx = g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                              : API::VR::get_right_controller_index();
        const bool have_hmd  = hidx >= 0 && get_pose(hidx, &hpos, &hq, false);
        const bool have_hand = cidx >= 0 && get_pose(cidx, &gpos, &gq, false);
        float hp = 0, hy = 0, hr = 0, gp = 0, gy = 0, gr = 0;
        if (have_hmd)  quat_to_rotator(-hq.z, hq.x, hq.y, -hq.w, &hp, &hy, &hr);
        if (have_hand) quat_to_rotator(-gq.z, gq.x, gq.y, -gq.w, &gp, &gy, &gr);
        const Vec3 head_ue{-hpos.z, hpos.x, hpos.y};
        const Vec3 hand_ue{-gpos.z, gpos.x, gpos.y};

        // Camera (Blam) and rendered view.
        double cp = 0.0, cy = 0.0;
        const bool have_cam = read_control_rotation(&cp, &cy, nullptr);
        const float view_yaw = halo::g_view_base_yaw.load();
        const float view_pit = halo::g_dbg_view_in_pitch.load();

        // Gun: where the FP mesh's weapon socket really is, world cm, and the mesh's rotation.
        g_tick_stage = "parent_frame";
        Vec3 gun_pos{}; Vec3 gun_rot{}; bool have_gun = false;
        if (auto* rigc = rig_tracked_component()) {
            have_gun = rig_socket_world(rigc, L"PrimaryWeapon", &gun_pos)
                    && call_ret_vec3(rigc, L"K2_GetComponentRotation", &gun_rot);
        }
        // Parent (camera component) world position, so gun can be read RELATIVE to the eye.
        Vec3 par_pos{}; bool have_par = (g_rig_parent != nullptr)
            && call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &par_pos);

        // Aim ray.
        const Vec3 aim_o{g_ret_origin.x, g_ret_origin.y, g_ret_origin.z};

        // THE JUDGE. hand-head is a room-frame vector; the room is pinned to the VIEW's yaw by the
        // lock, so rotating it by view yaw puts it in the WORLD -- the same frame gun_rel_eye is
        // read in. The two columns are then directly comparable, and their difference (err) is
        // the placement error in cm, measured, per second. No synthetic input can supply this;
        // only the real hand against the real gun socket does.
        // Same anchor the palette uses -- the STANDING ORIGIN, not the head -- or the judge would
        // report head motion as placement error while the palette (correctly) ignores it.
        const auto so_j = API::VR::get_standing_origin();
        const Vec3 so_ue{-so_j.z, so_j.x, so_j.y};
        const Vec3 hh{hand_ue.x - so_ue.x, hand_ue.y - so_ue.y, hand_ue.z - so_ue.z};
        const Vec3 hh_world = quat_rotate(rotator_to_quat(0.0f, view_yaw, 0.0f),
                                          Vec3{hh.x * 100.0f, hh.y * 100.0f, hh.z * 100.0f});
        const Vec3 gre{gun_pos.x - par_pos.x, gun_pos.y - par_pos.y, gun_pos.z - par_pos.z};
        const Vec3 err{gre.x - hh_world.x, gre.y - hh_world.y, gre.z - hh_world.z};

        if (pwl) API::get()->log_info(
            "[Halo-CampE-UEVR] TRACE head=(%.2f %.2f %.2f)m rot(p%.0f y%.0f r%.0f) | "
            "hand=(%.2f %.2f %.2f)m rot(p%.0f y%.0f r%.0f) hand-head=(%.2f %.2f %.2f)m | "
            "cam(p%.0f y%.0f) view(p%.0f y%.0f) | gun_socket_world=(%.0f %.0f %.0f)cm "
            "gun_rel_eye=(%.0f %.0f %.0f)cm mesh_rot(p%.0f y%.0f r%.0f) | "
            "aim_origin=(%.0f %.0f %.0f)cm | ok:hmd%d hand%d cam%d gun%d par%d",
            head_ue.x, head_ue.y, head_ue.z, hp, hy, hr,
            hand_ue.x, hand_ue.y, hand_ue.z, gp, gy, gr,
            hh.x, hh.y, hh.z,
            (float)cp, (float)cy, view_pit, view_yaw,
            gun_pos.x, gun_pos.y, gun_pos.z,
            gre.x, gre.y, gre.z,
            gun_rot.x, gun_rot.y, gun_rot.z,
            aim_o.x, aim_o.y, aim_o.z,
            (int)have_hmd, (int)have_hand, (int)have_cam, (int)have_gun, (int)have_par);
        // ERR expressed in the CAMERA's frame, so a term that rides the camera reads as a
        // constant column here instead of a swirl in world. And the two pitch numbers that decide
        // whether the residual is head-pitch, cam-pitch, or their difference, printed with it.
        const Vec3 err_cam = quat_rotate(quat_conj(rotator_to_quat((float)cp, (float)cy, 0.0f)), err);

        // THE SOCKET IN THE MESH'S OWN FRAME, next to what we WROTE to node 8. Four hand-frame
        // hypotheses have now failed against the JUDGE (cam yaw, view yaw, +-lock gap, hand-hmd),
        // and ERR_in_cam is a CONSTANT ~(0, +28, -15) cm at every pitch, gap and reach. A constant
        // camera-frame offset between the bone we place and the socket that renders is not a hand
        // frame error at all: it is an authored offset between the palette node and the mesh
        // socket. Measure it: pull the socket back into mesh-local (conj(mesh_rot) * (socket -
        // mesh_origin)) and print it beside node 8's written position, same frame, same units.
        // If they differ by a constant, that constant is the missing vector -- and it is measured.
        Vec3 sock_local{}; float w8x = 0, w8y = 0, w8z = 0;
        {
            const Quat mrq = rotator_to_quat(gun_rot.x, gun_rot.y, gun_rot.z);
            const Vec3 d{gun_pos.x - par_pos.x, gun_pos.y - par_pos.y, gun_pos.z - par_pos.z};
            const Vec3 sl = quat_rotate(quat_conj(mrq), d);
            // UE cm -> palette units, one Y flip, 304.8 -- the same map the write uses.
            sock_local = Vec3{sl.x / 304.8f, -sl.y / 304.8f, sl.z / 304.8f};
            w8x = halo::g_dbg_node8_x.load(); w8y = halo::g_dbg_node8_y.load(); w8z = halo::g_dbg_node8_z.load();
        }
        if (pwl) API::get()->log_info(
            "[Halo-CampE-UEVR] TRACE-NODE8 written=(%.3f %.3f %.3f)u  socket_in_mesh=(%.3f %.3f %.3f)u  "
            "diff=(%.3f %.3f %.3f)u = (%.0f %.0f %.0f)cm  [constant diff = authored socket offset]",
            w8x, w8y, w8z, sock_local.x, sock_local.y, sock_local.z,
            sock_local.x - w8x, sock_local.y - w8y, sock_local.z - w8z,
            (sock_local.x - w8x) * 304.8f, (sock_local.y - w8y) * 304.8f, (sock_local.z - w8z) * 304.8f);
        const std::string wkey_tr = weapon_key();
        if (pwl) API::get()->log_info(
            "[Halo-CampE-UEVR] TRACE-JUDGE hand_world=(%.1f %.1f %.1f)cm gun_rel_eye=(%.1f %.1f %.1f)cm "
            "ERR=(%.1f %.1f %.1f)cm |%.1f|cm  ERR_in_cam=(%.1f %.1f %.1f)cm  lock_gap=%.1f  "
            "cam_p=%.1f cam_y=%.1f head_p=%.1f head_y=%.1f hand_p=%.1f hand_y=%.1f dpitch=%.1f  reach=%.0fcm  th=%d  wpn=%s",
            hh_world.x, hh_world.y, hh_world.z, gre.x, gre.y, gre.z,
            err.x, err.y, err.z, std::sqrt(err.x*err.x + err.y*err.y + err.z*err.z),
            err_cam.x, err_cam.y, err_cam.z,
            wrap180((float)cy - view_yaw), (float)cp, (float)cy, hp, hy, gp, gy, wrap180((float)cp - hp),
            std::sqrt(hh_world.x*hh_world.x + hh_world.y*hh_world.y + hh_world.z*hh_world.z),
            (int)halo::two_hand_latched(),
            wkey_tr.empty() ? "-" : wkey_tr.c_str());

        // ---- THE BARREL. Position is proven; this is the gun's POINTING DIRECTION as rendered,
        // against the aim ray, in degrees. Read the socket's world rotation from the posed
        // skeleton, take its three axes, and print each one's angle to the aim ray -- the barrel
        // is whichever axis (or its negative) sits at a small constant angle. No assumption about
        // which authored axis is the barrel; the log picks it. Also the CONTROLLER's pointing
        // direction (through the rigid aim fix, lifted into the world by the view frame), so
        // barrel-vs-hand and aim-vs-hand are both numbers.
        {
            Vec3 srot{};
            const bool have_srot = (rig_tracked_component() != nullptr)
                && rig_socket_world_rot(rig_tracked_component(), L"PrimaryWeapon", &srot);
            const float ap = (float)cp * DEG2RAD, ay = (float)cy * DEG2RAD;
            const Vec3 aim_dir{std::cos(ap) * std::cos(ay), std::cos(ap) * std::sin(ay), std::sin(ap)};
            float hy_c = 0.0f, hp_c = 0.0f;
            const bool have_hand_dir = halo::derive_ctrl_angles(&hy_c, &hp_c,
                g_cfg.aim_left_hand ? API::VR::get_left_controller_index()
                                    : API::VR::get_right_controller_index());
            const float rigid_frame = g_cfg.view_lock ? g_locked_view_yaw.load() : 0.0f;
            const float hpr = hp_c * DEG2RAD, hyr = (hy_c + rigid_frame) * DEG2RAD;
            const Vec3 hand_dir{std::cos(hpr) * std::cos(hyr), std::cos(hpr) * std::sin(hyr), std::sin(hpr)};
            auto angdeg = [](const Vec3& a, const Vec3& b) {
                const float d = clampf(a.x*b.x + a.y*b.y + a.z*b.z, -1.0f, 1.0f);
                return std::acos(d) * RAD2DEG;
            };
            if (have_srot) {
                const Quat sq = rotator_to_quat(srot.x, srot.y, srot.z);
                const Vec3 sx = quat_rotate(sq, Vec3{1, 0, 0});
                const Vec3 sy = quat_rotate(sq, Vec3{0, 1, 0});
                const Vec3 sz = quat_rotate(sq, Vec3{0, 0, 1});
                // THE CORRECTION, SOLVED: the barrel is the socket's -Y (measured 19:27). Bring the
                // barrel and the aim ray into the palette pose's own frame and take the shortest
                // arc between them: that quaternion, right-multiplied onto the grip fix rotation,
                // puts the barrel ON the aim ray. Printed at full precision so it can be averaged
                // over rows and written to the file without anyone placing a hand on anything.
                {
                    const Quat P{halo::g_dbg_pose_w_x.load(), halo::g_dbg_pose_w_y.load(),
                                 halo::g_dbg_pose_w_z.load(), halo::g_dbg_pose_w_w.load()};
                    const Vec3 barrel_w{-sy.x, -sy.y, -sy.z};
                    const Vec3 b_l = quat_rotate(quat_conj(P), barrel_w);
                    const Vec3 a_l = quat_rotate(quat_conj(P), aim_dir);
                    // MEASURE THE BARREL AXIS IN THE POSE FRAME (a constant of the mesh) and
                    // publish it for the pullback's barrel lock. Still rows only -- during motion the
                    // socket readback and the published pose are a frame apart and b_l smears.
                    // EMA so one bad row cannot steer it; valid after a handful of samples.
                    if (have_hand_dir && angdeg(aim_dir, hand_dir) < 1.0f) {
                        static Vec3 s_bl{}; static int s_bl_n = 0;
                        // Once settled, a sample far from the mean is a weapon swap or a stale socket
                        // read (20:06:54: one 12-degree excursion during Magnum->AR), not the axis
                        // moving. Skip it rather than steer the lock through it.
                        const float sbn = std::sqrt(s_bl.x*s_bl.x + s_bl.y*s_bl.y + s_bl.z*s_bl.z);
                        const bool outlier = (s_bl_n >= 5 && sbn > 1.0e-3f)
                            && angdeg(Vec3{s_bl.x/sbn, s_bl.y/sbn, s_bl.z/sbn}, b_l) > 10.0f;
                        if (s_bl_n == 0) s_bl = b_l;
                        else if (!outlier) { const float a = 0.2f; s_bl = Vec3{s_bl.x + a*(b_l.x - s_bl.x), s_bl.y + a*(b_l.y - s_bl.y), s_bl.z + a*(b_l.z - s_bl.z)}; }
                        if (!outlier) ++s_bl_n;
                        const float bn = std::sqrt(s_bl.x*s_bl.x + s_bl.y*s_bl.y + s_bl.z*s_bl.z);
                        if (bn > 1.0e-3f && s_bl_n >= 5) {
                            halo::g_barrel_axis_x = s_bl.x / bn; halo::g_barrel_axis_y = s_bl.y / bn; halo::g_barrel_axis_z = s_bl.z / bn;
                            halo::g_barrel_axis_valid = true;
                        }
                        if ((s_bl_n % 20) == 5) {
                            if (g_cfg.palette_weapon_log) API::get()->log_info("[Halo-CampE-UEVR] BARRELAXIS in pose frame = (%.4f %.4f %.4f) n=%d  (lock %s)",
                                                 s_bl.x / bn, s_bl.y / bn, s_bl.z / bn, s_bl_n,
                                                 g_cfg.palette_barrel_lock ? "ON" : "off");
                        }
                    }
                    const Vec3 ax{b_l.y * a_l.z - b_l.z * a_l.y, b_l.z * a_l.x - b_l.x * a_l.z, b_l.x * a_l.y - b_l.y * a_l.x};
                    const float dt = b_l.x * a_l.x + b_l.y * a_l.y + b_l.z * a_l.z;
                    Quat dq{ax.x, ax.y, ax.z, 1.0f + dt};
                    const float dn = std::sqrt(dq.x*dq.x + dq.y*dq.y + dq.z*dq.z + dq.w*dq.w);
                    if (dn > 1.0e-6f) { dq.x /= dn; dq.y /= dn; dq.z /= dn; dq.w /= dn; }
                    if (g_cfg.palette_weapon_log) API::get()->log_info(
                        "[Halo-CampE-UEVR] TRACE-BARRELFIX delta_in_pose=(%.6f %.6f %.6f %.6f) = %.2f deg  "
                        "pose_w=(%.4f %.4f %.4f %.4f)  wpn=%s  [right-multiply onto gripfix rot]",
                        dq.x, dq.y, dq.z, dq.w, 2.0f * std::acos(clampf(std::fabs(dq.w), 0.0f, 1.0f)) * RAD2DEG,
                        P.x, P.y, P.z, P.w, wkey_tr.empty() ? "-" : wkey_tr.c_str());
                }
                if (g_cfg.palette_weapon_log) API::get()->log_info(
                    "[Halo-CampE-UEVR] TRACE-BARREL socket_rot(p%.1f y%.1f r%.1f) | aim(p%.1f y%.1f) "
                    "hand(p%.1f y%.1f)%s aim-hand=%.1fdeg | angle to AIM: +X %.1f +Y %.1f +Z %.1f "
                    "| angle to HAND: +X %.1f +Y %.1f +Z %.1f  wpn=%s  [barrel = the axis near 0 or 180]",
                    srot.x, srot.y, srot.z, (float)cp, (float)cy, hp_c, hy_c + rigid_frame,
                    have_hand_dir ? "" : "(no hand)", angdeg(aim_dir, hand_dir),
                    angdeg(sx, aim_dir), angdeg(sy, aim_dir), angdeg(sz, aim_dir),
                    angdeg(sx, hand_dir), angdeg(sy, hand_dir), angdeg(sz, hand_dir),
                    wkey_tr.empty() ? "-" : wkey_tr.c_str());
            } else {
                if (g_cfg.palette_weapon_log) API::get()->log_info("[Halo-CampE-UEVR] TRACE-BARREL socket rotation unavailable | aim(p%.1f y%.1f) hand(p%.1f y%.1f) aim-hand=%.1fdeg",
                    (float)cp, (float)cy, hp_c, hy_c + rigid_frame, angdeg(aim_dir, hand_dir));
            }
        }
    }

    // ---- MEASURE THE MESH CONSTANTS (see their declaration next to g_view_base_yaw).
    //
    // M and v0 relate the FP mesh's world transform to the aim rotator. They are measured, not
    // modelled, and re-measured every tick: if they hold still the model stands and the hook can
    // reconstruct the mesh transform FRESH from the aim rotator alone; if they drift with aim,
    // the drift line below says so and the model is dead. Reflected reads, so game thread only.
    g_tick_stage = "palette_weapon_frame";
    if (palette_weapon_mode()) {
        bool ok = false;
        auto* comp = rig_tracked_component();
        double cy = 0.0, cp = 0.0;
        if (comp != nullptr && g_rig_parent != nullptr &&
            read_control_rotation(&cp, &cy, nullptr)) {
            Vec3 crot{}, cpos{}, ppos{};
            if (call_ret_vec3(comp, L"K2_GetComponentRotation", &crot) &&
                call_ret_vec3(comp, L"K2_GetComponentLocation", &cpos) &&
                call_ret_vec3(g_rig_parent, L"K2_GetComponentLocation", &ppos)) {
                // ---- CLOSE THE BRACKET (Config.hpp mesh_const). The mesh rotation above was read
                // by a reflected call issued after the ControlRotation read, with two more
                // reflected calls between them, so `cp/cy` and `crot` describe different instants.
                // The mesh rides the camera, so that gap lands in M as the camera's angular
                // velocity times the gap -- exactly the judder, in the one factor every bone we
                // write shares. Read the camera a second time to MEASURE the gap instead of
                // assuming it away.
                double cp1 = cp, cy1 = cy;
                const bool brk_ok = read_control_rotation(&cp1, &cy1, nullptr);
                const float brk_dy = brk_ok ? wrap180((float)(cy1 - cy)) : 0.0f;
                const float brk_dp = brk_ok ? (float)(cp1 - cp) : 0.0f;
                const float brk_deg = std::sqrt(brk_dy * brk_dy + brk_dp * brk_dp);
                halo::g_meshM_brk_deg.store(brk_deg, std::memory_order_relaxed);

                static bool     s_have_M = false;
                static unsigned s_clean  = 0;
                const int  mcm   = g_cfg.mesh_const;
                const bool gated = (mcm == 1 || mcm == 3);
                const bool dirty = gated && brk_ok && brk_deg > g_cfg.mesh_const_gate;
                const bool frozen = (mcm == 3 && s_have_M && s_clean >= 8u);
                const bool accept = !dirty && !frozen;

                // Mode 2 pairs the mesh read with the MIDPOINT of the brackets, the best unbiased
                // estimate of where the camera was when the mesh rotation was actually sampled.
                const float use_p = (mcm == 2 && brk_ok) ? (float)(cp + 0.5 * brk_dp) : (float)cp;
                const float use_y = (mcm == 2 && brk_ok) ? (float)(cy + 0.5 * brk_dy) : (float)cy;
                const Quat cam  = rotator_to_quat(use_p, use_y, 0.0f);
                const Quat camI = quat_conj(cam);
                const Quat M    = quat_mul(camI, rotator_to_quat(crot.x, crot.y, crot.z));
                const Vec3 v0   = quat_rotate(camI, Vec3{cpos.x - ppos.x,
                                                         cpos.y - ppos.y,
                                                         cpos.z - ppos.z});
                halo::g_tick_seq.fetch_add(1u, std::memory_order_acq_rel);   // -> odd, writing
                if (accept) {
                    halo::g_meshM_x = M.x; halo::g_meshM_y = M.y;
                    halo::g_meshM_z = M.z; halo::g_meshM_w = M.w;
                    halo::g_meshV0_x = v0.x; halo::g_meshV0_y = v0.y; halo::g_meshV0_z = v0.z;
                    halo::g_meshM_acc_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_relaxed);
                    halo::g_meshM_acc.fetch_add(1, std::memory_order_relaxed);
                    s_have_M = true;
                    if (!dirty) ++s_clean;
                } else {
                    halo::g_meshM_rej.fetch_add(1, std::memory_order_relaxed);
                }
                // The CURRENT tick's camera and mesh rotation keep publishing every tick either
                // way. They describe this tick and several freshness gates downstream read
                // g_tick_cam_ms; starving them would break modes 8 and 9 rather than fix M.
                halo::g_tick_cam_p.store((float)cp, std::memory_order_relaxed);
                halo::g_tick_cam_y.store((float)cy, std::memory_order_relaxed);
                {
                    const Quat mq = rotator_to_quat(crot.x, crot.y, crot.z);
                    halo::g_tick_mrot_x.store(mq.x, std::memory_order_relaxed);
                    halo::g_tick_mrot_y.store(mq.y, std::memory_order_relaxed);
                    halo::g_tick_mrot_z.store(mq.z, std::memory_order_relaxed);
                    halo::g_tick_mrot_w.store(mq.w, std::memory_order_relaxed);
                }
                halo::g_tick_cam_ms.store((long long)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count(), std::memory_order_release);
                halo::g_tick_seq.fetch_add(1u, std::memory_order_acq_rel);   // -> even, readable
                // A rejected sample is not a failure: M is a constant and the last clean one still
                // stands. Only "never seeded at all" is invalid.
                ok = s_have_M || (mcm == 0);
                if (mcm == 0) {
                    halo::g_meshM_x = M.x; halo::g_meshM_y = M.y;
                    halo::g_meshM_z = M.z; halo::g_meshM_w = M.w;
                    halo::g_meshV0_x = v0.x; halo::g_meshV0_y = v0.y; halo::g_meshV0_z = v0.z;
                }

                // CONSTANCY IS THE CLAIM -- print the evidence. Worst deviation from the first
                // accepted sample, in degrees and cm, over each window.
                if (g_cfg.palette_weapon_log) {
                    static Quat  s_M0{};  static Vec3 s_v00{};
                    static bool  s_seeded = false;
                    static float s_worst_deg = 0.0f, s_worst_cm = 0.0f;
                    static uint32_t s_last_rep = 0;
                    if (!s_seeded) { s_M0 = M; s_v00 = v0; s_seeded = true; }
                    const float dot = clampf(std::fabs(M.x*s_M0.x + M.y*s_M0.y +
                                                       M.z*s_M0.z + M.w*s_M0.w), 0.0f, 1.0f);
                    const float ddeg = 2.0f * std::acos(dot) * RAD2DEG;
                    const Vec3  dv{v0.x - s_v00.x, v0.y - s_v00.y, v0.z - s_v00.z};
                    const float dcm = std::sqrt(dv.x*dv.x + dv.y*dv.y + dv.z*dv.z);
                    if (ddeg > s_worst_deg) s_worst_deg = ddeg;
                    if (dcm  > s_worst_cm)  s_worst_cm  = dcm;
                    if (tick - s_last_rep >= 320) {
                        s_last_rep = tick;
                        // THE VALUES. A "constant" that is constant and WRONG is invisible to a
                        // drift meter, and M has never once been printed as a value. If M is not
                        // identity, the pullback's conj(cam*M) and the lift's cam do NOT cancel,
                        // and the residual rotates with the camera -- which is what the JUDGE
                        // shows: the gun swinging +-60 cm while the hand moves 4.
                        float mp = 0.0f, my = 0.0f, mr = 0.0f;
                        quat_to_rotator(M.x, M.y, M.z, M.w, &mp, &my, &mr);
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] MESHCONST M=(p%.1f y%.1f r%.1f) v0=(%.1f %.1f %.1f)cm "
                            "comp_rot=(p%.1f y%.1f r%.1f) cam=(p%.1f y%.1f) | drift M %.1f deg v0 %.1f cm"
                            " | mode %d bracket %.3f deg acc %u rej %u",
                            mp, my, mr, v0.x, v0.y, v0.z, crot.x, crot.y, crot.z, cp, cy,
                            s_worst_deg, s_worst_cm, g_cfg.mesh_const, brk_deg,
                            halo::g_meshM_acc.load(std::memory_order_relaxed),
                            halo::g_meshM_rej.load(std::memory_order_relaxed));
                        s_worst_deg = s_worst_cm = 0.0f;
                    }
                }
            }
        }
        halo::g_mesh_const_valid.store(ok, std::memory_order_release);
    }

    // ---- THE AXIS-PROBE SAMPLER (see palette_probe in Config.hpp and the probe in
    // BlamPalette.cpp). Accumulates the rendered weapon's world position per probe phase and, at
    // each cycle boundary, prints the three measured columns: bone axis -> world direction, in
    // cm per palette unit. The player stands still; the probe does the moving.
    if (palette_weapon_mode() && g_cfg.palette_probe) {
        static Vec3     s_sum[6] = {};
        static uint32_t s_cnt[6] = {};
        static int      s_prev_phase = -1;

        const int phase = blam_palette_probe_phase();
        auto* rigc = rig_tracked_component();
        Vec3 sock{};
        const bool sock_ok = (phase >= 0 && rigc != nullptr &&
                              rig_socket_world(rigc, L"PrimaryWeapon", &sock));
        // HEARTBEAT, OUTSIDE every gate -- the two silent sessions happened because the failing
        // gate was also the gate on the evidence. phase -1 = the hook never ran the probe (hook
        // dead, or the held weapon builds under a different slot than the phase clock counts);
        // sock_ok 0 = the readback is failing; counts frozen = the settle/steady gates eat all.
        if (g_cfg.palette_weapon_log) {
            static uint32_t s_hb = 0;
            static Vec3     s_hb_sum[6] = {};   // referenced below; zeroed alias for clarity
            (void)s_hb_sum;
            if ((++s_hb % 160u) == 0u) {
                API::get()->log_info(
                    "[Halo-CampE-UEVR] PALETTEPROBE phase=%d rigc=%d sock_ok=%d",
                    phase, (int)(rigc != nullptr), (int)sock_ok);
            }
        }
        if (sock_ok) {
            // THE RENDER FLICKERS between stock and displaced during odd phases -- the heartbeat
            // proved it (baseline bins 170+, displaced bins 0-3: a steadiness filter rejected the
            // displaced state as motion, because it IS motion, at frame rate). So classify
            // instead of filter: during a displaced phase, the just-completed baseline's mean is
            // the anchor, and only samples further than 25 cm from it -- the displaced mode of
            // the bimodal flicker -- are accumulated. Baselines accumulate everything after the
            // settle. The player's own drift is handled by the anchor being LOCAL to each
            // baseline/displaced pair rather than global to the cycle.
            static uint32_t s_phase_age = 0;
            static Vec3     s_anchor{};
            static bool     s_anchor_ok = false;
            if (phase != s_prev_phase) {
                if ((phase & 1) == 1 && s_prev_phase == phase - 1 &&
                    s_cnt[s_prev_phase] >= 8) {
                    s_anchor = Vec3{s_sum[s_prev_phase].x / s_cnt[s_prev_phase],
                                    s_sum[s_prev_phase].y / s_cnt[s_prev_phase],
                                    s_sum[s_prev_phase].z / s_cnt[s_prev_phase]};
                    s_anchor_ok = true;
                } else if ((phase & 1) == 1) {
                    s_anchor_ok = false;
                }
                s_phase_age = 0;
            } else {
                ++s_phase_age;
            }
            if (s_phase_age > 20) {
                if ((phase & 1) == 0) {
                    s_sum[phase].x += sock.x; s_sum[phase].y += sock.y; s_sum[phase].z += sock.z;
                    s_cnt[phase]++;
                } else if (s_anchor_ok) {
                    const Vec3 d{sock.x - s_anchor.x, sock.y - s_anchor.y, sock.z - s_anchor.z};
                    if ((d.x*d.x + d.y*d.y + d.z*d.z) > 625.0f) {   // >25 cm from baseline
                        s_sum[phase].x += sock.x; s_sum[phase].y += sock.y;
                        s_sum[phase].z += sock.z;
                        s_cnt[phase]++;
                    }
                }
            }
            // HEARTBEAT: where do samples die? Two silent sessions in a row were spent inferring
            // that from the outside; this prints it. phase -1 = the hook is not running the probe
            // at all (wrong slot? hook dead?); counts stuck at 0 with a live phase = the settle or
            // steady gate is eating everything.
            if (g_cfg.palette_weapon_log) {
                static uint32_t s_hb = 0;
                if ((++s_hb % 160u) == 0u) {
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] PALETTEPROBE phase=%d age=%u counts=%u/%u/%u/%u/%u/%u",
                        phase, s_phase_age,
                        s_cnt[0], s_cnt[1], s_cnt[2], s_cnt[3], s_cnt[4], s_cnt[5]);
                }
            }

            // Cycle boundary: phase wrapped from 5 back to 0 with data in every bin.
            if (phase == 0 && s_prev_phase == 5) {
                bool full = true;
                for (int i = 0; i < 6; ++i) full = full && (s_cnt[i] >= 8);
                if (full) {
                    const float amt = blam_palette_probe_amount();
                    Vec3 m[6];
                    for (int i = 0; i < 6; ++i) {
                        m[i] = Vec3{s_sum[i].x / s_cnt[i], s_sum[i].y / s_cnt[i],
                                    s_sum[i].z / s_cnt[i]};
                    }
                    for (int a = 0; a < 3; ++a) {
                        const Vec3 col{(m[2*a+1].x - m[2*a].x) / amt,
                                       (m[2*a+1].y - m[2*a].y) / amt,
                                       (m[2*a+1].z - m[2*a].z) / amt};
                        API::get()->log_info(
                            "[Halo-CampE-UEVR] PALETTEAXIS bone %c -> world (%+7.1f %+7.1f %+7.1f) cm/unit",
                            'X' + a, col.x, col.y, col.z);
                    }
                    // The camera at capture, so the CONSTANT part of the map can be separated
                    // from wherever the player happened to be facing: bridge = conj(cam*M) * B.
                    double pcp = 0.0, pcy = 0.0;
                    read_control_rotation(&pcp, &pcy, nullptr);
                    API::get()->log_info(
                        "[Halo-CampE-UEVR] PALETTEAXIS cam=(p%.1f y%.1f) M=(%.4f %.4f %.4f %.4f)",
                        pcp, pcy,
                        halo::g_meshM_x.load(), halo::g_meshM_y.load(),
                        halo::g_meshM_z.load(), halo::g_meshM_w.load());
                }
                for (int i = 0; i < 6; ++i) { s_sum[i] = Vec3{}; s_cnt[i] = 0; }
            }
            s_prev_phase = phase;
        }
    }
