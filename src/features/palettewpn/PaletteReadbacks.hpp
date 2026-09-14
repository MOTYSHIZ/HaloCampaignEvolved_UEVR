#pragma once

// THE PALETTE WEAPON'S READBACKS (palettewpn): the posed skeleton's socket, the per-weapon rigid delta
// and the Blam control record's angles, for the palette instruments and its placement.

#include "Config.hpp"   // WeaponFix
#include "Math.hpp"     // Vec3
#include "uevr/API.hpp"

#include <string>

namespace halo {

// The WORLD position of a socket on the rig, in cm. Where derive_pivot answers "where is the grip
// relative to the component", this answers "where did the grip actually END UP" -- which is the
// only honest way to score a palette write, because it is read back from the posed skeleton rather
// than computed from the values we hoped we wrote. Reflected call: GAME THREAD ONLY.
bool rig_socket_world(uevr::API::UObject* rig, const wchar_t* socket, Vec3* out);
// The socket's WORLD rotation (pitch, yaw, roll degrees) from the posed skeleton.
bool rig_socket_world_rot(uevr::API::UObject* rig, const wchar_t* socket, Vec3* out_pyr);

// PALETTE per-weapon rigid delta (Config::WeaponFix): the entry whose match is a substring of the
// key, or nullptr. LAST match wins, the same rule weapon_fix_for() applies, so a captured entry
// outranks the shipped baseline it sits behind in the table. Consumed by BlamPalette.cpp.
const WeaponFix* wpnfix_find(const std::string& key);

// FRAMEAUDIT: the control record's angles in UE-convention degrees (false if unresolved).
bool blam_ctl_read_ue_deg(float* yaw_deg, float* pitch_deg);

// The Blam aim writer's record write (the sim thread, ~2600 calls/s): the writer agreement note and
// STOMPLOG point 23. The palettewpn table's sim_record_written slot.
void palette_wpn_sim_record_written(float yaw, float pitch);

} // namespace halo
