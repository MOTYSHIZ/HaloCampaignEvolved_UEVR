// palettewpn (fork feature, Experimental): the palette publishes (mesh constants, tick camera, barrel axis, debug poses) and the STOMPLOG ring.
// Textual fragment, included by Plugin.cpp in its namespace halo publish block. Moved verbatim; not compiled on its own.
// The rendered-view pitch, published beside the yaw for the palette's frame math.
std::atomic<float> g_dbg_view_in_pitch{0.0f};

// The yaw the stereo callback actually OUTPUT this frame -- the rendered base the palette's lock
// correction divides against. Stored on every path through the callback (passthrough when the
// lock is off or unprimed, the assigned value when it holds), so the palette maths reads what was
// really rendered instead of rebuilding locked+turn from parts. In namespace halo for
// BlamPalette.cpp; written from the render thread, read on the game tick -- one frame of
// staleness during a snap turn, invisible next to the turn itself.

// ---- THE MESH-VS-CAMERA CONSTANTS, for the palette's world-space pullback.
//
// The sweep proved the palette's camera error is a TRANSLATION: the FP mesh hangs off the
// rotating camera on a lever, so aim swings sweep the whole bone frame through the world --
// metre-scale, unfixable by any rotation of the bones. The fix is to aim the bones at a WORLD
// target (the same one rigmode's proven math produces) and pull it back through the mesh's
// actual world transform. That transform decomposes as
//     comp_rot = cam * M                where M  = conj(cam) * comp_rot     (rotation constant)
//     comp_pos = parent_pos + cam * v0  where v0 = conj(cam) * (comp - parent)  (lever constant)
// with cam = the aim rotator -- the ONE volatile term, which the sim-side hook can read FRESH at
// build time. M and v0 are constants of the rig (mesh-authoring yaw, camera-to-mesh lever),
// measured here on the game thread where reflected reads are possible, published for the hook.
// If they are NOT constant the model is wrong, so their drift is measured and printed too -- a
// drifting "constant" is a theory failing loudly, which is the only acceptable way left.
std::atomic<float> g_meshM_x{0.0f}, g_meshM_y{0.0f}, g_meshM_z{0.0f}, g_meshM_w{1.0f};
std::atomic<float> g_meshV0_x{0.0f}, g_meshV0_y{0.0f}, g_meshV0_z{0.0f};
// The GAME THREAD's camera, published each tick from the mesh-constant block below -- the same
// read, same thread, same moment as the transform the FP mesh is built from. The palette build
// consumes THIS (palettecam=6) instead of reading ControlRotation from the sim thread at a
// different moment: the right-hand-only flicks were the gap between those two moments.
std::atomic<float> g_tick_cam_p{0.0f}, g_tick_cam_y{0.0f};
std::atomic<long long> g_tick_cam_ms{0};
// The mesh's OWN rotation at the same game-thread instant (palettecam=8's source): the one
// epoch-consistent sample of the transform the tick will render, no render-thread K2 read.
std::atomic<float> g_tick_mrot_x{0.0f}, g_tick_mrot_y{0.0f}, g_tick_mrot_z{0.0f}, g_tick_mrot_w{1.0f};
// MESHCONST instrument (Config.hpp mesh_const): how far the camera moved BETWEEN the two
// ControlRotation reads that bracket the reflected mesh read, in degrees. This is the
// contamination in M, measured directly, per tick. g_meshM_age_ms says how old the M currently
// in use is, so a held-but-clean M cannot be mistaken for a fresh one.
// The tick publishes THREE things that only mean anything together, cam_tick, M and the mesh
// rotation Q_tick, and until now they were three unsynchronised relaxed stores. A reader that
// straddles a tick gets cam from tick N with M from tick N-1, and since M = conj(cam) x Q the
// product then misses by exactly one tick of camera motion, which is the same magnitude as the
// judder. Odd seq means a write is in progress. Same doctrine as g_p_seq.
std::atomic<unsigned> g_tick_seq{0};
std::atomic<float> g_meshM_brk_deg{0.0f};
std::atomic<long long> g_meshM_acc_ms{0};
std::atomic<unsigned> g_meshM_acc{0}, g_meshM_rej{0};
// The engine tick counter, bumped at tick start -- palbuildgate=4 keys "first build this tick"
// on it, and it costs one relaxed add whether or not anything reads it.
std::atomic<unsigned> g_tick_id{0};

// ---- STOMPLOG (Config.hpp stomp_log). Points: 0 = engine-tick start, 1 = stereo pre eye 0,
// 2 = stereo pre eye 1, 3 = stereo post eye 0. Single ring; concurrent writers use an atomic
// index; a torn row is one bad sample in a hunt instrument.
namespace stomplog {
struct Row { double t_ms; int point; float yaw; float e0; float e1; float e2; };
// CIRCULAR since 2026-09-11: the ring holds the LAST ~100 s instead of the first, so the key
// can stay ON all session and a flush (key to 0, or teardown) dumps the freshest window --
// "all of the logging is on" as a standing state, not a 60-second appointment.
constexpr int kCap = 60000;
Row g_rows[kCap];
std::atomic<int> g_n{0};
std::atomic<bool> g_was_on{false};
int g_seq = 0;
double now_ms() {
    return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace stomplog
void stomp_flush() {
    if (stomplog::g_was_on.load(std::memory_order_relaxed)) {
        {
            const int n = stomplog::g_n.exchange(0, std::memory_order_relaxed);
            stomplog::g_was_on.store(false, std::memory_order_relaxed);
            if (n > 0 && g_cfg_path[0] != '\0') {
                char path[MAX_PATH]; strncpy_s(path, sizeof(path), g_cfg_path, _TRUNCATE);
                char* slash = strrchr(path, '\\');
                if (slash != nullptr) {
                    char leaf[64]; sprintf_s(leaf, "halo_vr_stomp_%03d.csv", stomplog::g_seq++);
                    slash[1] = '\0'; strncat_s(path, sizeof(path), leaf, _TRUNCATE);
                    FILE* f = nullptr;
                    if (fopen_s(&f, path, "wb") == 0 && f != nullptr) {
                        fprintf(f, "t_ms,point,yaw,e0,e1,e2\r\n");
                        const int m = n < stomplog::kCap ? n : stomplog::kCap;
                        const int start = (n > stomplog::kCap) ? (n % stomplog::kCap) : 0;
                        for (int i = 0; i < m; ++i) {
                            const stomplog::Row& rw = stomplog::g_rows[(start + i) % stomplog::kCap];
                            fprintf(f, "%.3f,%d,%.4f,%.4f,%.4f,%.4f\r\n", rw.t_ms, rw.point, rw.yaw, rw.e0, rw.e1, rw.e2);
                        }
                        fclose(f);
                        API::get()->log_info("[Halo-CampE-UEVR] STOMPLOG: %d samples -> %hs", m, leaf);
                    }
                }
            }
        }
    }
}
// The periodic variant: the mode-6 test was lost when the rolling ring overwrote it during
// typing time -- evidence must land on disk BEFORE it can age out. Ring snapshot on the game
// thread (~1.5 MB copy, sub-millisecond), CSV written by a detached thread. Torn rows from
// writers racing the copy are a few bad samples in a hunt instrument.
static void stomp_flush_async() {
    const int n = stomplog::g_n.exchange(0, std::memory_order_relaxed);
    if (n <= 0 || g_cfg_path[0] == '\0') return;
    const int m = n < stomplog::kCap ? n : stomplog::kCap;
    const int start = (n > stomplog::kCap) ? (n % stomplog::kCap) : 0;
    auto* buf = new stomplog::Row[m];
    for (int i = 0; i < m; ++i) buf[i] = stomplog::g_rows[(start + i) % stomplog::kCap];
    char path[MAX_PATH]; strncpy_s(path, sizeof(path), g_cfg_path, _TRUNCATE);
    char* slash = strrchr(path, '\\');
    if (slash == nullptr) { delete[] buf; return; }
    char leaf[64]; sprintf_s(leaf, "halo_vr_stomp_%03d.csv", stomplog::g_seq++);
    slash[1] = '\0'; strncat_s(path, sizeof(path), leaf, _TRUNCATE);
    std::string spath(path);
    std::thread([buf, m, spath]() {
        FILE* f = nullptr;
        if (fopen_s(&f, spath.c_str(), "wb") == 0 && f != nullptr) {
            fprintf(f, "t_ms,point,yaw,e0,e1,e2\r\n");
            for (int i = 0; i < m; ++i)
                fprintf(f, "%.3f,%d,%.4f,%.4f,%.4f,%.4f\r\n", buf[i].t_ms, buf[i].point, buf[i].yaw, buf[i].e0, buf[i].e1, buf[i].e2);
            fclose(f);
            API::get()->log_info("[Halo-CampE-UEVR] STOMPLOG auto-flush: %d samples -> %hs", m, spath.c_str());
        } else { }
        delete[] buf;
    }).detach();
}
static void stomp_sample(int point) {
    if (g_cfg.stomp_log == 0) { stomp_flush(); return; }
    if (point == 0 && stomplog::g_n.load(std::memory_order_relaxed) >= (stomplog::kCap * 4) / 5)
        stomp_flush_async();
    stomplog::g_was_on.store(true, std::memory_order_relaxed);
    auto* comp = rig_tracked_component();
    if (comp == nullptr) return;
    Vec3 crot{};
    if (!call_ret_vec3(comp, L"K2_GetComponentRotation", &crot)) return;
    const Quat M{halo::g_meshM_x.load(std::memory_order_relaxed), halo::g_meshM_y.load(std::memory_order_relaxed),
                       halo::g_meshM_z.load(std::memory_order_relaxed), halo::g_meshM_w.load(std::memory_order_relaxed)};
    const Quat cam = quat_mul(rotator_to_quat(crot.x, crot.y, crot.z), quat_conj(M));
    float p = 0.0f, y = 0.0f, r = 0.0f;
    quat_to_rotator(cam.x, cam.y, cam.z, cam.w, &p, &y, &r);
    // Beside the mesh camera: the tick id and ControlRotation AT THIS SAME INSTANT, so the CSV
    // shows when the game updates each of the three within the tick, not just the mesh.
    double scp = 0.0, scy = 0.0;
    if (!read_control_rotation_hook(&scp, &scy)) { scp = 0.0; scy = 0.0; }
    const int i = stomplog::g_n.fetch_add(1, std::memory_order_relaxed);
    stomplog::g_rows[i % stomplog::kCap] = stomplog::Row{stomplog::now_ms(), point, y,
        (float)g_tick_id.load(std::memory_order_relaxed), (float)scy, (float)scp};
}
// The SIM-THREAD entry into the same ring (hooked_pose stamps its builds here). No UObject
// reads -- the caller supplies the values -- so it is safe from any thread; the atomic index
// makes ring order a true arrival order across threads. Points 4..7 = build calls,
// 4 + weapon_slot*2 + (capture ? 0 : 1). Flushing stays with stomp_sample on the tick.
void stomp_mark(int point, float yaw, float e0, float e1, float e2) {
    if (g_cfg.stomp_log == 0) return;
    const int i = stomplog::g_n.fetch_add(1, std::memory_order_relaxed);
    stomplog::g_rows[i % stomplog::kCap] = stomplog::Row{stomplog::now_ms(), point, yaw, e0, e1, e2};
}
std::atomic<bool>  g_mesh_const_valid{false};
// Where the hook last WROTE node 8 (palette units), published so the game thread's TRACE-NODE8
// can put it beside the socket read back from the posed skeleton, in the same frame.
std::atomic<float> g_dbg_node8_x{0.0f}, g_dbg_node8_y{0.0f}, g_dbg_node8_z{0.0f};
// The palette's WORLD pose as last resolved (view-lifted, grip fix included, before the camera
// divide), published so TRACE-BARREL can solve the barrel->aim correction in the pose's own frame.
std::atomic<float> g_dbg_pose_w_x{0.0f}, g_dbg_pose_w_y{0.0f}, g_dbg_pose_w_z{0.0f}, g_dbg_pose_w_w{1.0f};
// The barrel axis IN THE PALETTE POSE'S FRAME: conj(pose_w) * (socket -Y in world), measured on the
// game thread from the posed skeleton and averaged over still rows. A constant of the mesh, not
// of the player. Consumed by the pullback's barrel lock (see Config::palette_barrel_lock).
std::atomic<float> g_barrel_axis_x{0.0f}, g_barrel_axis_y{0.0f}, g_barrel_axis_z{0.0f};
std::atomic<bool>  g_barrel_axis_valid{false};
