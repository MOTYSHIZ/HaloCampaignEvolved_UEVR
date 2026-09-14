// reloadvr (fork feature, Experimental): the fetch hand pose and its dead-pose gate.
// Textual fragment, included by Gesture.cpp inside reload_update(), after the fetch hand index. Moved verbatim; not compiled on its own.
    // No head pose, no belt or well to measure against: the fetch hand counts as absent.
    bool have_left = (head_p != nullptr) && get_pose(lidx, &hand_l, &lrot, /*use_aim=*/false);
    // DEAD-POSE GATE. get_pose passes a sleeping/glitched controller as (0,0,0) -- the room origin,
    // which sits at the head on this rig. From there the well is within 30 cm and "lifted" reads
    // +60 cm, so ONE bad frame seats the magazine: logged 2026-08-16 12:32 as a seat 164 ms after
    // a grab that was 67 cm from the well. Same rule the palette publisher applies to the aim hand.
    if (have_left) {
        const Vec3& head = *head_p;
        const float rx = hand_l.x - head.x, ry = hand_l.y - head.y, rz = hand_l.z - head.z;
        const float reach2 = rx*rx + ry*ry + rz*rz;
        const bool zero = std::fabs(hand_l.x) < 1e-6f && std::fabs(hand_l.y) < 1e-6f && std::fabs(hand_l.z) < 1e-6f;
        if (zero || !std::isfinite(reach2) || reach2 > 1.5f * 1.5f) {
            have_left = false;
            if (g_cfg.reload_log) {
                static uint32_t s_dead = 0;
                if ((s_dead++ % 30u) == 0u)
                    API::get()->log_info("[Halo-CampE-UEVR] RELOAD: ignoring dead/implausible left pose (zero=%d reach=%.2fm)", (int)zero, std::sqrt(reach2));
            }
        }
    }
