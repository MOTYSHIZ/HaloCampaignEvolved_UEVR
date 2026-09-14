#include "core/fixes/ReticuleFixes.hpp"

#include "Config.hpp"
#include "Reticule.hpp"   // g_ret_widget_comp, reticule_mode3_reassert

#include <Windows.h>

#include <map>
#include <string>

namespace halo {

namespace {
std::map<std::string, ULONGLONG> s_load_failed;   // path -> when it last came back null
}
bool load_asset_recently_failed(const char* path) {
    const auto it = s_load_failed.find(path);
    if (it == s_load_failed.end()) return false;
    if (GetTickCount64() - it->second < 300000ull) return true;
    s_load_failed.erase(it);
    return false;
}
void load_asset_remember_failure(const char* path) { s_load_failed[path] = GetTickCount64(); }

const char* load_asset_log_suffix(uevr::API::UObject* obj) {
    return obj ? "" : " (remembered: not retried for 5 min)";
}

void load_asset_note_result(const char* path, uevr::API::UObject* obj) {
    if (obj == nullptr) load_asset_remember_failure(path);
}

bool widget_log_enabled() {
    return g_cfg.widget_log;
}

void reticule_widget_moved() {
    reticule_mode3_reassert();
}

bool widget_alpha_hide_applies(uevr::API::UObject* comp) {
    return comp == g_ret_widget_comp.get();
}

} // namespace halo
