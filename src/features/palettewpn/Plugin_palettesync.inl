// palettewpn (fork feature, Experimental): hand and aim captured together for the placement (palettesync).
// Textual fragment, included by Plugin.cpp inside the XInput hook render-rate law, after the aim sample. Moved verbatim; not compiled on its own.
            // PALETTESYNC (Config.hpp palette_sync): hand and aim sampled together, here, where the
            // aim law samples them, and handed to the placement as one pair.
            if (g_cfg.palette_sync && palette_weapon_mode()) {
                const int32_t sidx = g_aim_law_ridx.load();
                Vec3 sap{}, sgp{}; Quat saq{}, sgq{};
                if (sidx >= 0 && get_pose(sidx, &sap, &saq, /*use_aim=*/true) &&
                    get_pose(sidx, &sgp, &sgq, /*use_aim=*/false)) {
                    halo::blam_palette_sync_capture(saq.x, saq.y, saq.z, saq.w, sap.x, sap.y, sap.z,
                                                    sgq.x, sgq.y, sgq.z, sgq.w, sgp.x, sgp.y, sgp.z,
                                                    (float)ap, (float)ay);
                }
            }
