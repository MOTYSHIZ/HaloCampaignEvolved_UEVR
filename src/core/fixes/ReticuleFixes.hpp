#pragma once

// FIXES IN THE RETICULE MODULE that belong to no feature. Each runs from a hook point in Reticule.cpp
// (see features/hooks/ReticuleHooks.hpp) and runs whether or not any feature is on.

#include <string_view>

#include "uevr/API.hpp"

namespace halo {

// A FAILED LOAD IS REMEMBERED (2026-09-06): the nav markers are rebuilt on every weapon swap and every
// menu, and each rebuild re-ran the blocking disk load for a material this install does not ship --
// 124 times in thirteen minutes, a freeze every six seconds. A path that came back null is not tried
// again for five minutes.
bool load_asset_recently_failed(const char* path);
void load_asset_remember_failure(const char* path);
// The load log line's suffix: empty on success, the remembered-failure note on a null load.
const char* load_asset_log_suffix(uevr::API::UObject* obj);
// Right after the load log line: remember a null load.
void load_asset_note_result(const char* path, uevr::API::UObject* obj);

// The widget-tint probe log in bind_widget_slate_ui runs only with widgetlog on.
bool widget_log_enabled();

// Late re-assert for xrlayerhidews modes 3/4 at the end of reticule_widget_move: it runs after
// anything earlier in the tick that rebuilt the component. No-op unless xrlayer=1 and xrlayerhidews
// is 3 or 4.
void reticule_widget_moved();

// HIDE BY ALPHA only for the reticule widget itself. Every other host shares the tint function for
// its gain (the wrist HUD panels do), and applying the hide to all of them made the whole wrist HUD
// transparent the moment the layer went live -- and kept it so after the reticule was switched off,
// because the hidden state cannot clear without a bound reticule widget.
bool widget_alpha_hide_applies(uevr::API::UObject* comp);

} // namespace halo
