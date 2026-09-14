// vehcam (fork feature, Experimental): the seat publish evidence counters and the cached rider/vehicle objects.
// Textual fragment, included by BlamDrive.cpp at namespace halo scope, with the unit-state publishes. Moved verbatim; not compiled on its own.
std::atomic<uint32_t> g_seat_pub_seq{0}, g_seat_pub_calls{0}, g_seat_norec{0}, g_seat_reresolve{0},
                      g_seat_direct_reads{0};
std::atomic<uintptr_t> g_seat_obj{0}, g_seat_vobj{0};
std::atomic<uint32_t>  g_seat_vdat{0xFFFFFFFFu};
