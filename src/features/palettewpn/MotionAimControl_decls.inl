// palettewpn (fork feature, Experimental): the pose latch, draw-aim intent and writer agreement declarations.
// Textual fragment, included by MotionAimControl.hpp inside namespace halo. Moved verbatim; not compiled on its own.
// POSELATCH: snapshot every device once so all readers share one sample. site 1 = engine tick
// start, site 3 = the aim law's XInput sample. See the definition for the modes.
void pose_latch_refresh(int site);
// DRAWAIM: hand intent (UE degrees) of the outgoing snapshot (prev) and the new one (cur), stored
// at each latch refresh. palettecam 14 divides by prev, 15 by cur.
extern std::atomic<float> g_intent_prev_y, g_intent_prev_p, g_intent_cur_y, g_intent_cur_p;
extern std::atomic<bool>  g_intent_prev_ok, g_intent_cur_ok;
extern std::atomic<float> g_intent_prev2_y, g_intent_prev2_p;   // RETSTAMP: intent two snapshots back
extern std::atomic<bool>  g_intent_prev2_ok;
// FRAMEAUDIT: snapshot generation counter, the generation of the stored prev intent, and the
// generation the CALLING THREAD was last served by get_pose.
extern std::atomic<uint32_t> g_latch_gen, g_intent_prev_gen, g_intent_cur_gen;
uint32_t pose_latch_last_gen();
// The Blam aim writer notes the UE-convention angle it wrote, for the AIMWRITERS agreement line.
void aim_writer_note_blam(float yaw_deg, float pitch_deg);
