#pragma once

// HOOK POINTS IN Scope.cpp. Definitions: src/features/FeatureList.cpp.

namespace halo {

// scope_handle_lt (the XInput hook's thread), right after the menu/seat refusal. True = a feature
// owns the scope, so the left trigger must not toggle the pane; the feature has already cleared the
// trigger edge state it was handed and the pane's active flag, and scope_handle_lt returns false.
bool features_scope_trigger_stood_down(bool& s_down);

// scope_handle_button (the XInput hook's thread), inside its refusal condition, after scope_enabled.
// True = a feature owns the scope, so the bound button must not toggle the pane.
bool features_scope_pane_stands_down();

} // namespace halo
