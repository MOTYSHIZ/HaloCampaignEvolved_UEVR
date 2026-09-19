#include "core/config/KeyAlias.hpp"

#include <cstring>

namespace halo {
namespace {

struct KeyAliasRow { const char* old_name; const char* new_name; };

// Retired name -> current name, feature by feature. Adding a row is how a fork key is renamed.
const KeyAliasRow kAliases[] = {
    // palettewpn -- stem "palette". The twohand* six carried the AUTHOR'S stem and read as his.
    { "twohandmarker",        "palettetwohandmarker" },
    { "twohandmarkerscale",   "palettetwohandmarkerscale" },
    { "twohandmarkercolor",   "palettetwohandmarkercolor" },
    { "twohandrad",           "palettetwohandrad" },
    { "twohandblendms",       "palettetwohandblendms" },
    { "twohandhaptic",        "palettetwohandhaptic" },
    { "aimdirectwrite",       "paletteaimdirectwrite" },
    { "aimreticulefresh",     "paletteaimreticulefresh" },
    { "slidewatch",           "paletteslidewatch" },
    { "slidezonepriority",    "paletteslidezonepriority" },
    { "meshconst",            "palettemeshconst" },
    { "meshconstgate",        "palettemeshconstgate" },
    { "poselatch",            "paletteposelatch" },
    { "palbuildgate",         "palettebuildgate" },
    { "palpubframe",          "palettepubframe" },
    { "palwpncalibkey",       "palettewpncalibkey" },
    // roomscale -- stem "roomscale". blam* is the author's own family of aim-hook keys.
    { "blamthrottleysign",    "roomscalethrottleysign" },
    { "blamunitthrottleoff",  "roomscalethrottleoff" },
    { "blamunitthrottleoff2", "roomscalethrottleoff2" },
    // holsterpollthrow -- stem "holsterpollthrow", the master key itself: holsterthrow* is
    // already the author's (holsterthrowspeed), so the feature has no shorter stem of its own.
    { "gripmaskl",            "holsterpollthrowgripmaskl" },
    { "gripmaskr",            "holsterpollthrowgripmaskr" },
    { "grenhand",             "holsterpollthrowhand" },
    { "greninstant",          "holsterpollthrowinstant" },
    { "grenspeed",            "holsterpollthrowspeed" },
    { "grenbackdate",         "holsterpollthrowbackdate" },
    { "throwdump",            "holsterpollthrowdump" },
    { "holsterthrowlog",      "holsterpollthrowlog" },
    // wristhud -- stem "wrist". hud* is the author's flat-HUD family.
    { "hudplacement",         "wristhudplacement" },
    { "hudwpnammo",           "wristhudwpnammo" },
    { "hudwpnanchor",         "wristhudwpnanchor" },
    { "hudwpnfallback",       "wristhudwpnfallback" },
    { "hudwpngap",            "wristhudwpngap" },
    { "hudwpngrenade",        "wristhudwpngrenade" },
    { "hudwpnlog",            "wristhudwpnlog" },
    { "hudwpnscale",          "wristhudwpnscale" },
    { "hudwpnshield",         "wristhudwpnshield" },
    { "hudwpntracker",        "wristhudwpntracker" },
    { "blipcolor",            "wristblipcolor" },
    { "blipcolorother",       "wristblipcolorother" },
    { "blipname",             "wristblipname" },
    { "blipbytes",            "wristblipbytes" },
    { "blipdump",             "wristblipdump" },
    { "trackerdump",          "wristtrackerdump" },
    { "trackermid",           "wristtrackermid" },
    // stabilityfixes -- stem "stability".
    { "turnlog",              "stabilityturnlog" },
    { "widgetlog",            "stabilitywidgetlog" },
    { "holstermarkercolor",   "stabilityholstermarkercolor" },
    { "grenminthrow",         "stabilitygrenminthrow" },
    // vehiclewheel -- stem "vehwheel". veh* alone is the seat camera's.
    { "vehsteersign",         "vehwheelsteersign" },
    // reloadvr -- stem "reload".
    { "roomanchor",           "reloadroomanchor" },
    { "coophide",             "reloadcoophide" },
    { "hidesolo",             "reloadhidesolo" },
    { "akmimic",              "reloadakmimic" },
    { "shotgunlog",           "reloadshotgunlog" },
};

constexpr int kAliasCount = (int)(sizeof(kAliases) / sizeof(kAliases[0]));

} // namespace

const char* key_current_name(const char* key) {
    if (key == nullptr) return key;
    for (int i = 0; i < kAliasCount; ++i)
        if (_stricmp(key, kAliases[i].old_name) == 0) return kAliases[i].new_name;
    return key;
}

void key_alias_append_reference(std::string& text) {
    if (!text.empty() && text.back() != '\n') text += "\r\n";
    text += "\r\n# ============================================================ RETIRED KEY NAMES (generated)\r\n"
            "# Fork keys renamed so that every one of them starts with its feature's stem. The name on the\r\n"
            "# left still parses and lands in exactly the same field with the same clamp; the name on the\r\n"
            "# right is the one the catalogs, the settings menu and the logs use. Nothing is ever written\r\n"
            "# back under an old name, so a file keeps whichever spelling you typed.\r\n"
            "#\r\n";
    for (int i = 0; i < kAliasCount; ++i) {
        text += "#   ";
        text += kAliases[i].old_name;
        text += "  ->  ";
        text += kAliases[i].new_name;
        text += "\r\n";
    }
}

} // namespace halo
