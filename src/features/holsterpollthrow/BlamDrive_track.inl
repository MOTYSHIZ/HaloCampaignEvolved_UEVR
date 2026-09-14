// holsterpollthrow (fork feature, Experimental): the throw dump header, now_ms() (also used by the wrist radar scan later in this file) and the grenade-track handoff.
// Textual fragment, included by core/UnitState.cpp at namespace halo scope, after the unit-state comment. Moved verbatim; not compiled on its own.
// ---- THROW WINDUP DUMP (throwdump, doctrine in Config.hpp). SIM THREAD. Statics only, no
// allocation; the cost while idle is one memcmp-sized pass over 0x600 bytes per sim call, and the
// log lines are capped per window. The mask is learned, not assumed: anything that churns while
// nothing is being thrown is by definition not the throw.
inline long long now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// GRENTRACK handoff (see BlamDrive.hpp): written by the dev spawn hook, sampled below.
std::atomic<uintptr_t> g_grentrack_obj{0};
std::atomic<long long> g_grentrack_at_ms{0};
std::atomic<uintptr_t> g_greninst_obj{0};
std::atomic<long long> g_greninst_at_ms{0};
std::atomic<float>     g_greninst_vx{0.0f}, g_greninst_vy{0.0f}, g_greninst_vz{0.0f};
