// vehcam (fork feature, Experimental): the seat publish evidence and vehseatdirect declarations.
// Textual fragment, included by BlamDrive.hpp inside namespace halo. Moved verbatim; not compiled on its own.
// SEAT PUBLISH EVIDENCE (vehlog). seq advances on every successful rider read from any path (sim
// publish or direct read), so the camera can tell a live rider from a frozen one. calls = sim
// publishes, norec = stick-mode calls that had no control record to publish from, reresolve =
// stick-mode record re-resolves, direct = vehseatdirect reads.
extern std::atomic<uint32_t> g_seat_pub_seq, g_seat_pub_calls, g_seat_norec, g_seat_reresolve,
                             g_seat_direct_reads;
// The rider object and its vehicle object as the last sim publish saw them (vehseatdirect).
extern std::atomic<uintptr_t> g_seat_obj, g_seat_vobj;

// vehseatdirect: refresh the seat atomics from the cached object pointers. Any thread; acts only
// while stick mode holds the sim publish's normal path off. No-op when the key is 0.
void seat_direct_refresh();
