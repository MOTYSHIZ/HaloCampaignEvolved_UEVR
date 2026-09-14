// holsterpollthrow (fork feature, Experimental): the grenade-track and instant-release exports.
// Textual fragment, included by BlamDrive.hpp inside namespace halo. Moved verbatim; not compiled on its own.
// GRENTRACK (dev, throwdump): the projectile object the spawn hook just created, and when. The
// hook (sim thread) writes them; throw_dump_probe samples the object's position for ~1.2 s so the
// log shows whether the grenade FLIES from spawn or sits held until an animation event.
extern std::atomic<uintptr_t> g_grentrack_obj;
extern std::atomic<long long> g_grentrack_at_ms;

// GRENINSTANT (dev, greninstant): the grenade released at spawn, and the velocity to keep
// re-asserting on it for 400 ms so the animation keyframe's own late release is overwritten.
extern std::atomic<uintptr_t> g_greninst_obj;
extern std::atomic<long long> g_greninst_at_ms;
extern std::atomic<float>     g_greninst_vx, g_greninst_vy, g_greninst_vz;
