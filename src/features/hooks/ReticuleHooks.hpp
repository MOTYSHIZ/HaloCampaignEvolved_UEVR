#pragma once

// HOOK POINTS IN Reticule.cpp. Definitions: src/features/FeatureList.cpp. Each runs on the thread of
// the Reticule.cpp function it sits in (the game thread for the reticule's own paths; the render
// thread when the wrist HUD tints its panels).

#include <string_view>

#include "uevr/API.hpp"

namespace halo {

// load_asset_by_path, right after the path check: true = this path failed within five minutes, the
// load is skipped (core fix).
bool features_asset_load_recently_failed(const char* path);

// load_asset_by_path, the load log line's suffix, and right after it the failure is remembered (core
// fix). The suffix is empty on success.
const char* features_asset_load_suffix(uevr::API::UObject* obj);
void features_asset_load_done(const char* path, uevr::API::UObject* obj);

// bind_widget_slate_ui, in the tint probe log's condition, before the 1-in-32 counter: stabilitywidgetlog.
bool features_widget_log();

// apply_widget_tint, in the gain: the multiplier the calling host asked for (1 for every call the
// author's reticule makes).
float features_widget_tint_mul();

// apply_widget_tint, as the last term of the alpha hide: whether the hide applies to this component
// (core fix: the reticule widget only).
bool features_widget_alpha_hide_applies(uevr::API::UObject* comp);

// reticule_widget_move, at its end (core fix: the mode 3/4 late re-assert).
void features_reticule_widget_moved();

} // namespace halo
