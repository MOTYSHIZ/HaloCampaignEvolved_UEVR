// reloadvr (fork feature, Experimental): the per-weapon belt point and the stale-survey test for the belt magazine.
// Textual fragment, included by Holster.cpp in its anonymous namespace, after the magazine survey state. Moved verbatim; not compiled on its own.
// The belt point for the weapon in hand: a reloadmagoffw entry if one matches, the global
// reloadmagoff otherwise. Both the rendered mag and the grab zone read this, so what you see and
// what you reach for stay the same point.
Vec3 mag_belt_point() {
    Vec3 mo{g_cfg.reload_mag_off[0], g_cfg.reload_mag_off[1], g_cfg.reload_mag_off[2]};
    const std::string key = weapon_key();
    if (key.empty() || g_cfg.reload_mag_off_w[0] == 0) return mo;
    std::string lk = key; for (auto& ch : lk) ch = (char)tolower((unsigned char)ch);
    const std::string tbl = g_cfg.reload_mag_off_w;
    size_t pos = 0;
    while (pos <= tbl.size()) {
        size_t comma = tbl.find(',', pos); if (comma == std::string::npos) comma = tbl.size();
        std::string ent = tbl.substr(pos, comma - pos);
        while (!ent.empty() && (unsigned char)ent.back()  <= ' ') ent.pop_back();
        while (!ent.empty() && (unsigned char)ent.front() <= ' ') ent.erase(ent.begin());
        const size_t colon = ent.find(':');
        if (colon != std::string::npos && colon > 0) {
            std::string name = ent.substr(0, colon);
            for (auto& ch : name) ch = (char)tolower((unsigned char)ch);
            if (lk.find(name) != std::string::npos) {
                float x = mo.x, y = mo.y, z = mo.z;
                if (sscanf_s(ent.c_str() + colon + 1, "%f/%f/%f", &x, &y, &z) == 3) mo = Vec3{x, y, z};
                return mo;
            }
        }
        if (comma >= tbl.size()) break;
        pos = comma + 1;
    }
    return mo;
}

// The survey HAS candidates and every one is a dead handle -- a level transition recycled them.
// The caller must re-survey on that, or the fallback chain bottoms out at the frag mesh and the
// belt mag renders as a grenade (field report, 2026-08-31). An empty list is "never surveyed".
bool mag_cands_stale() {
    if (s_mag_cands.empty()) return false;
    for (auto& cnd : s_mag_cands) if (cnd.obj.get() != nullptr) return false;
    return true;
}
