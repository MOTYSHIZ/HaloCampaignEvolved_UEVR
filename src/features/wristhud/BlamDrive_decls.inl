// wristhud (fork feature, Experimental): the radar blip exports.
// Textual fragment, included by BlamDrive.hpp inside namespace halo. Moved verbatim; not compiled on its own.
// ---- WRIST RADAR blips (blipdump survey, 2026-08-28): unit+0x177 is the TEAM byte -- 0x0E
// human (player + marines, armed or corpse), 0x0D covenant (the one carrier photographed held
// PLASMA grenades). Published by the sim (slow cached table scan + per-publish position reads):
// relative Blam-unit offsets from the player, team, and a moving flag. Consumed by WristHud.
constexpr int MAX_BLIPS = 12;
extern std::atomic<int>   g_blip_count;
extern std::atomic<float> g_blip_dx[MAX_BLIPS], g_blip_dy[MAX_BLIPS];
extern std::atomic<int>   g_blip_team[MAX_BLIPS];     // 0 = human, 1 = covenant
extern std::atomic<bool>  g_blip_moving[MAX_BLIPS];
// Identity (low dword of the object pointer): publish order compacts as contacts drop in and
// out of range, so an INDEX is not a contact -- the renderer's per-blip smoothing must key on
// this or it smears one dot's motion onto another's.
extern std::atomic<uint32_t> g_blip_id[MAX_BLIPS];
// The RAW +0x177 byte, published alongside the two-way classification. The original survey saw
// three humans and one Covenant, which is far too thin to call the byte "team" -- different
// enemies paint different colours in the field, so at least one more value exists. Logged so the
// real value set can be read off instead of assumed.
extern std::atomic<uint32_t> g_blip_raw[MAX_BLIPS];
// SPECIES ID: the object's leading dword, a tag/definition id. THIS is the real species key --
// surveyed 2026-08-29, it groups instances exactly (six of one type, two of another) and it
// separates a MARINE from an ELITE, which +0x177 cannot (both read 0x0E there). Blip colour is
// keyed on this. Assumed stable across runs; if colours ever shuffle between sessions, re-survey
// -- a per-run pointer or handle would look just like this in a single capture.
extern std::atomic<uint32_t> g_blip_type[MAX_BLIPS];
// The movement test's own numbers per contact: measured speed (blam units/sec), the window it
// was measured over (ms), and the consecutive-window run. Published because contacts that were
// visibly walking metres reported mv=0, and the test has to be read rather than reasoned about.
// ABSOLUTE Blam position per contact, for the species-naming match: a contact is identified by
// finding the Unreal actor standing at the same place and reading its CLASS NAME, which -- unlike
// the tag id -- is the same on every level.
extern std::atomic<float> g_blip_wx[MAX_BLIPS], g_blip_wy[MAX_BLIPS], g_blip_wz[MAX_BLIPS];
extern std::atomic<float> g_blip_speed[MAX_BLIPS];
extern std::atomic<int>   g_blip_dtm[MAX_BLIPS];
extern std::atomic<int>   g_blip_run[MAX_BLIPS];
