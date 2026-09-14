// holsterpollthrow (fork feature, Experimental): the standalone spawn hook for the grenade windup capture (throwdump, dev).
// Textual fragment, included by BlamAim.cpp at namespace halo scope, before blam_aim_tick(). Moved verbatim; not compiled on its own.
// THE SPAWN HOOK ALONE, for the grenade-windup capture. blamaim=1 proved unusable for this: it
// takes ownership of the aim-write function, which (a) replaces months of shipping aim with the
// investigation-era law -- "aim completely off" in the headset -- and (b) stands down BlamDrive's
// hook, killing publish_unit_state and with it the THROWDUMP probe. One session produced spawn
// rows with frozen aim and no press marks: worthless twice over. This installs ONLY the
// create_projectile hook, driven by the same `throwdump` key as the probe, so press timeline and
// spawn timestamps come from one session with normal aim.
void blam_spawnlog_tick() {
    if (g_cfg.blam_aim != 0) return;   // blamaim owns both hooks; stand down to it entirely
    const bool want = g_cfg.throw_dump != 0;
    if (!want) {
        if (g_create_hook_id >= 0 && g_hook_id < 0) {   // ours, not blamaim's
            API::get()->param()->functions->unregister_inline_hook(g_create_hook_id);
            g_create_hook_id = -1;
            g_orig_create = nullptr;
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSPAWN: removed (throwdump off)");
        }
        return;
    }
    if (g_create_hook_id >= 0) return;
    HMODULE sim = GetModuleHandleA("HaloSimulation_tag_release.dll");
    if (sim == nullptr) return;
    g_sim_base = (uintptr_t)sim;
    void* ctarget = (void*)(g_sim_base + RVA_CREATE_PROJECTILE);
    static bool s_refused = false;   // one refusal line, not one per tick
    if (IsBadReadPtr(ctarget, sizeof(CREATE_PROJECTILE_PROLOGUE)) ||
        memcmp(ctarget, CREATE_PROJECTILE_PROLOGUE, sizeof(CREATE_PROJECTILE_PROLOGUE)) != 0) {
        if (!s_refused) {
            s_refused = true;
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSPAWN: prologue mismatch at dll+0x%llX -- "
                                 "the game moved; spawn hook stays off",
                                 (unsigned long long)RVA_CREATE_PROJECTILE);
        }
        return;
    }
    const int cid = API::get()->param()->functions->register_inline_hook(
        ctarget, (void*)&hooked_create_projectile, (void**)&g_orig_create);
    if (cid < 0 || g_orig_create == nullptr) {
        if (!s_refused) {
            s_refused = true;
            API::get()->log_info("[Halo-CampE-UEVR] BLAMSPAWN: hook FAILED (id=%d)", cid);
        }
        return;
    }
    g_create_hook_id = cid;
    API::get()->log_info("[Halo-CampE-UEVR] BLAMSPAWN: installed standalone on 0x%llX (dll+0x%llX) id=%d",
                         (unsigned long long)ctarget, (unsigned long long)RVA_CREATE_PROJECTILE, cid);
}
