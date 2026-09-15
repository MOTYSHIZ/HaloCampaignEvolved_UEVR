#pragma once

// THE PALETTE WEAPON'S CALIBRATION (palettewpn): its own keys (palgripfix, palaimfix, palaimoffyaw, palaimoffpitch,
// palaimcalibver, palwpnfix, palwpncalibkey) and its own file, halo_vr_palette_calib.cfg. The author's calibration
// keys and files are neither read nor written here.

namespace halo {

// palettewpn's parse slot, for the keys above. True = taken.
bool palette_calib_parse_key(const char* key, const char* val, double v);

} // namespace halo
