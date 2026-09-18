// The palette arm driver, as the rest of the plugin sees it.
//
// ⚠️ UNVERIFIED AGAINST A RUNNING GAME. Nothing in this module has been watched working: the offset
// chain in PaletteHook.cpp was measured on someone else's copy of the simulation DLL, and the
// frame conversion below has never been checked against a real palette. It is default-OFF
// (armdriver=0) and it exists to be A/B'd against the UE route in a headset, not to be shipped.
//
// EXCLUSIVITY. Never call these directly from a tick. ArmDriver.hpp arbitrates; this module only
// runs when arm_driver_owns(ArmDriverMode::Palette) is true, because the UE route drives the same
// visible arms and two drivers must never both have them.

#pragma once

#include <atomic>    // g_pa_torso_yaw / g_pa_torso_seq
#include <cstdint>

namespace halo {

// Game thread, once per tick. Installs or removes the hook to match the mode, advances the
// two-handed hold, and republishes tuning from config. The actual palette work happens later, on
// the game's own thread, inside the detour.
void palettearm_update(float delta_seconds);

// Remove the hook and drop all state. Safe to call when nothing is installed.
void palettearm_release();

// Consume the palettearm config keys. Called from Config.cpp's parse chain; returns true when the
// key belonged to this module.
//
// Hoisted out of the main chain for the same reason parse_scope_key() was: MSVC counts every
// `else if` toward its 128-deep block limit and the chain is already near it (fatal C1061).
bool palettearm_parse_key(const char* key, double value);

// One line of status for the support diagnostics: resolution, install state, call count.
const char* palettearm_status();

// Second status line: the last drive's geometry -- root, wrist target, achieved wrist, and
// how far the solver missed by. For diagnosing a pose that is WRONG rather than absent.
const char* palettearm_status_geom();
// Palette-space position of the AIM arm's shoulder node after anchoring (Blam units, root frame),
// for the world-space probe to log beside the rendered bones. Zeros until the drive has run.
void palettearm_dbg_shoulder(float* x, float* y, float* z);
// The aim arm's shoulder / elbow / achieved wrist after the solve (palette units, root frame).
void palettearm_dbg_arm(float sh[3], float el[3], float wr[3]);

// THE UeRig WEAPON SOLUTION, handed to the palette route (pawpnrig). Game thread, once per tick,
// from the rig block. Everything is in the BODY frame -- level, yaw = locked view yaw + turn, origin
// at the rig's attach parent (the camera) -- in UE axes (+X forward, +Y right, +Z up) and UE cm:
//   fwd/right/up  the images of the mesh's axes under rig mode's mesh rotation q_gun
//   wpn_cm        where rig mode puts the weapon (the PrimaryWeapon attach point)
// valid=false whenever rig mode itself would not place the gun (origin hold, a rig mode other than
// 3). Ages out on its own if the rig block stops running.
// frozen = a weapon calibration hold is pinning the gun in the world: the target is still valid for
// the carry, but it no longer describes how the gun sits on the controller.
void palettearm_note_rig_weapon(bool valid, const float fwd[3], const float right[3],
                                const float up[3], const float wpn_cm[3], bool frozen);
// The STOCK weapon marker this frame -- the weapon socket's offset from the FP model's root as
// rig mode measures it (the Magnum: (61.2,13.1,-23.5) cm measured on the rig, (61.5,-13.1,-23.5)
// on the stock palette; UE's Y is Blam's -Y) -- in UE axes and centimetres. False when the live
// drive has not run in the last quarter second. The scope's virtual rig frame is built from it.
bool palettearm_stock_marker_ue(float out_cm[3]);
// ...and the marker's REST pose in the same frame and units (learned, remembered or baked -- see
// RecoilPass); false until one is known. The scope's socket handshake judges "the bone is at rest"
// against this rather than against a slow average of its own, which after a swap was still
// remembering the previous weapon for seconds.
// The optional axes are the rest pose's BASIS in the same UE model frame: the marker's own X, Y
// and Z axes as unit vectors (the socket's frame, since the PrimaryWeapon socket IS this node).
// With them the socket-relative placement of anything mounted in the rig frame is a closed form,
// M^-1 (t - S) -- no world reads, no clock to be a tick behind.
bool palettearm_stock_marker_rest_ue(float out_cm[3], float x_axis[3] = nullptr, float y_axis[3] = nullptr,
                                     float z_axis[3] = nullptr);
// Bumped every time the weapon MODEL changes (the tag the palette keys its memory by), so a
// consumer sampling the marker knows its history is from another weapon.
int  palettearm_model_serial();
// True while the palette is actually carrying the weapon bone to the rig target (this live frame,
// within the last quarter second). False at spawn until the carry is up, during an origin hold,
// and whenever the live drive is not running: the socket then sits at the stock animation's place.
bool palettearm_carry_active();
const char* palettearm_status_jitter();

// Has this route given up for the session?
//
// True after either terminal failure: the builder could not be resolved in a LOADED simulation
// module (deterministic -- the same scan fails identically every tick), or the hook installed and
// the watchdog then proved it is never called. ArmDriver.cpp reads this and falls back to the UE
// route, because the alternative is far worse than it sounds: with the palette route selected and
// broken, the UE route is also standing down, so the weapon is not controller-attached and the
// arms are stock. That state is strictly worse than never having enabled the palette route, and
// nothing recovers from it on its own.
//
// Latched in a session-local static, deliberately NOT written back to g_cfg: load_config() resets
// the whole struct every ~2 s, so a config field cannot carry a runtime decision -- the file value
// would come straight back and this would become a retry-and-log loop. BlamDrive.cpp records the
// same trap.
bool palettearm_unavailable();

// The solved torso yaw in degrees -- the frame the shoulders hang from -- and a publish counter.
// Published so the tick can answer "does this basis stay with the body, or follow the aim?" against
// the two view yaws, which keep internal linkage in Plugin.cpp. Only meaningful while the palette
// arm driver is running. Check the counter before trusting the yaw: a stale yaw reads as a torso
// that is perfectly still, which is the answer the A/B is hoping for.
extern std::atomic<float>    g_pa_torso_yaw;
extern std::atomic<uint32_t> g_pa_torso_seq;

// True when the palette owns the weapon, so the legacy MESH drive must stand down -- otherwise the
// container displaces the gun and the palette displaces it again, and the two compose.
//
// MODE-LEVEL, NOT PER-FRAME, and deliberately so. Keying this off "did we actually write the branch
// this frame" would hand the mesh back on any frame the reach guard rejected, and an ownership flag
// that flickers is what made the wpndrive engage edge stomp the arms fifteen times in one session.
// Ownership is a decision about configuration; success is a decision about a frame.
bool palettearm_weapon_owns();

// Does the palette route keep its OWN weapon calibration (wpnfix, captured by its own freeze)?
//
// NOT under pawpnrig. There the gun is placed by the RIG's solution, so the rig's fitted grip and
// mount ARE the weapon's chain, and the rig's gestures -- the menu/End pose match, the per-weapon
// wpnoff capture -- are the ones that move it. Found headless 2026-09-17, and reported from a headset
// the same night as "it tends to snap back and not save the value": WeaponCalib claimed every release
// for the palette driver on the old premise that the rig's calibration was not in the chain, so the
// solve ran, nothing was written or adopted, and WeaponOffset put the old fit back on the next tick.
// ONE predicate for both sides of the hand-over, for the reason palettearm_weapon_owns() gives.
bool palettearm_weapon_calib_owns();

// What the game is about to be TOLD on the pad, after every remap and injection of ours: is the
// melee / swap-weapon / throw-grenade mask down? Called from the XInput hook each poll; any thread.
// The palette arms key the melee-animation preference, the equip hand-over and the grenade tail
// off these, because none of those actions can be told apart from the pose alone.
void palettearm_note_pad(bool melee_down, bool swap_down, bool throw_down, bool sprint_down,
                         bool moving, bool reload_down);



// Can the SUPPORT-HAND calibration gesture do anything?
//
//   0 = NOT CONFIGURED -- this driver is not posing the support hand at all (a different armdriver,
//       the support arm switched off in paarms, or the route gave up). The feature does not exist
//       in that configuration, and the shipped arm driver is exactly that configuration.
//   1 = configured, but nothing has been posed yet: no weapon, no tracking, or still at the
//       frontend. Nothing to freeze YET.
//   2 = ready.
//
// TWO STATES AND NOT ONE, because the menu must treat them differently: at 0 it hides the entry
// entirely (a player on the shipped configuration should not be shown a control that cannot work),
// while at 1 it shows the entry and explains what is missing. Collapsing them would either
// advertise the feature to everyone or hide it from the people who have it.
//
// Published to the menu through the status file the bridge writes, so the panel never arms a mode
// nothing will ever look at.
int palettearm_support_hand_ready();

// Clear the give-up latch and try again. Called by the arbiter when the armdriver key CHANGES, so
// toggling the key away and back re-arms the route without restarting the game.
void palettearm_retry();

} // namespace halo
