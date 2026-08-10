// Aim-loop trace recorder. See AimTrace.hpp for why this buffers instead of writing.

#include "AimTrace.hpp"

#if HALO_VR_DEV

#include "Config.hpp"
#include "MotionAimControl.hpp"

#include <Windows.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>

namespace halo {
namespace {

// Statically allocated on purpose. Allocating on arm would be tidier about memory, but it would
// put a new[] in a race with a sampler that may already be running on the input thread; a fixed
// array has no lifetime to get wrong. ~4 MB, dev builds only, and at render rate this still holds
// about four minutes -- far longer than any single tuning run.
constexpr int MAX_SAMPLES = 120000;
AimTraceSample    g_ring[MAX_SAMPLES];

std::atomic<int>  g_count{0};
std::atomic<bool> g_armed{false};
std::atomic<bool> g_overflow{false};
std::chrono::steady_clock::time_point g_t0;

bool g_prev_flag = false;   // tick-thread only
int  g_seq       = 0;       // tick-thread only

// Build a sibling path to the config file, so traces land next to the profile the harness is
// already reading and writing.
void sibling_path(char* out, size_t n, const char* leaf) {
    out[0] = '\0';
    if (g_cfg_path[0] == '\0') return;
    strncpy_s(out, n, g_cfg_path, _TRUNCATE);
    char* slash = strrchr(out, '\\');
    if (slash) slash[1] = '\0'; else out[0] = '\0';
    strncat_s(out, n, leaf, _TRUNCATE);
}

void write_csv() {
    const int n = g_count.load();
    if (n <= 0) return;

    char tmp[MAX_PATH], fin[MAX_PATH];
    char leaf[64];
    sprintf_s(leaf, "halo_vr_trace_%03d.csv.tmp", g_seq);
    sibling_path(tmp, sizeof(tmp), leaf);
    sprintf_s(leaf, "halo_vr_trace_%03d.csv", g_seq);
    sibling_path(fin, sizeof(fin), leaf);
    if (tmp[0] == '\0') return;

    FILE* f = nullptr;
    if (fopen_s(&f, tmp, "wb") != 0 || f == nullptr) return;

    // The config that produced this trace goes in the header, so a result can never be separated
    // from the settings that caused it -- the single easiest way to ruin a tuning run is to
    // mis-attribute a good trace to the wrong configuration.
    fprintf(f, "# halo_vr aim trace %03d\r\n", g_seq);
    fprintf(f, "# samples=%d overflow=%d\r\n", n, (int)g_overflow.load());
    fprintf(f, "# ffgain=%.4f dgain=%.4f ffsmoothms=%.2f dsmoothms=%.2f\r\n",
            g_cfg.ff_gain, g_cfg.d_gain, g_cfg.ff_smooth_ms, g_cfg.d_smooth_ms);
    fprintf(f, "# full=%.4f floor=%.4f dead=%.4f deadhyst=%.4f max=%.4f\r\n",
            g_cfg.full_deg, g_cfg.floor, g_cfg.dead_deg, g_cfg.dead_hyst, g_cfg.max_out);
    fprintf(f, "t,dt,des_yaw,des_pitch,aim_yaw,aim_pitch,out_rx,out_ry,ff_rate_yaw,meas_rate\r\n");

    for (int i = 0; i < n; ++i) {
        const AimTraceSample& s = g_ring[i];
        fprintf(f, "%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.4f,%.4f\r\n",
                s.t, s.dt, s.des_yaw, s.des_pitch, s.aim_yaw, s.aim_pitch,
                s.out_rx, s.out_ry, s.ff_rate_yaw, s.meas_rate);
    }
    fclose(f);

    // Rename last: the harness polls for the final name, so it can never read a half-written file.
    DeleteFileA(fin);
    MoveFileA(tmp, fin);
}

} // namespace

void aim_trace_sample(float dt, float des_yaw, float des_pitch,
                      double aim_yaw, double aim_pitch,
                      float out_rx, float out_ry, float ff_rate_yaw) {
    if (!g_armed.load(std::memory_order_relaxed)) return;

    const int i = g_count.fetch_add(1, std::memory_order_relaxed);
    if (i >= MAX_SAMPLES) {
        g_count.store(MAX_SAMPLES, std::memory_order_relaxed);   // stop counting past the end
        g_overflow.store(true, std::memory_order_relaxed);
        return;
    }

    // Truncation rather than wrap-around: a tuning run cares about the stimulus from its start,
    // and a ring that overwrote the beginning would silently return a trace of the wrong window.
    AimTraceSample& s = g_ring[i];
    s.t = std::chrono::duration<float>(std::chrono::steady_clock::now() - g_t0).count();
    s.dt = dt;
    s.des_yaw = des_yaw;  s.des_pitch = des_pitch;
    s.aim_yaw = (float)aim_yaw;  s.aim_pitch = (float)aim_pitch;
    s.out_rx = out_rx;  s.out_ry = out_ry;
    s.ff_rate_yaw = ff_rate_yaw;
    s.meas_rate = g_meas_rate.load(std::memory_order_relaxed);
}

void aim_trace_tick() {
    const bool want = g_cfg.aim_trace;
    if (want == g_prev_flag) return;
    g_prev_flag = want;

    if (want) {
        g_count.store(0);
        g_overflow.store(false);
        g_t0 = std::chrono::steady_clock::now();
        g_armed.store(true);      // last: nothing samples into a ring that is still being reset
    } else {
        g_armed.store(false);     // first: stop sampling before reading the ring
        ++g_seq;
        write_csv();
    }
}

} // namespace halo

#endif // HALO_VR_DEV
