// roomscale (fork feature, Experimental): the cross-module publishes (camera bob, throttle command, pad magnitude, facing probe).
// Textual fragment, included by Plugin.cpp in its namespace halo publish block. Moved verbatim; not compiled on its own.
// CAMERA BOB: the fast part of (camera - pawn root), world cm, published by the game tick.
// Subtracted from the palette's world hand so view and gun stay consistent.
std::atomic<float> g_bob_x{0.0f}, g_bob_y{0.0f}, g_bob_z{0.0f};
// ROOMSCALE THROTTLE (Config::roomscale_throttle): the movement command the game tick wants
// written into the unit object, in the AIM frame (forward, right), 0..1. Consumed on the SIM
// THREAD by BlamDrive; g_rs_thr_written counts writes so the tick can tell "the command
// reached the unit" from "nothing drove" when crediting eye travel.
std::atomic<float>    g_rs_thr_fwd{0.0f}, g_rs_thr_right{0.0f};
std::atomic<bool>     g_rs_thr_active{false};
std::atomic<uint32_t> g_rs_thr_written{0};
// The player's raw stick magnitude this poll, for the sim-side yield check: a deliberate push
// is locomotion, the throttle write waits.
std::atomic<float> g_pad_user_mag{0.0f};
// Candidate body-facing vectors for the throttle-frame probe log (see roomscale_thr_probe).
std::atomic<float> g_dbg_face[6]{};
