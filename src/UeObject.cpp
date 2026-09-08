#include "UeObject.hpp"

#include <cstring>

using namespace uevr;

namespace halo {

std::string narrow(const std::wstring& w) {
    std::string s;
    s.reserve(w.size());
    for (wchar_t c : w) s.push_back((c > 0 && c < 127) ? (char)c : '?');
    return s;
}

std::wstring class_name_of(API::UObject* obj) {
    if (obj == nullptr) return L"";
    auto* c = obj->get_class();
    if (c == nullptr) return L"";
    auto* f = c->get_fname();
    if (f == nullptr) return L"";
    return f->to_string();
}

bool uobject_live(API::UObject* p, int32_t* cached_index) {
    if (p == nullptr || cached_index == nullptr) return false;
    auto* arr = API::get()->get_uobject_array();
    if (arr == nullptr) return true;                       // cannot tell -- fail open
    const int32_t n = arr->get_object_count();

    // Fast path: the slot we already know still holds it.
    if (*cached_index >= 0 && *cached_index < n && arr->get_object(*cached_index) == p) {
        return true;
    }
    // Either first sight of this pointer, or the slot changed. One walk to (re)locate it;
    // not finding it means the object is gone.
    for (int32_t i = 0; i < n; ++i) {
        if (arr->get_object(i) == p) { *cached_index = i; return true; }
    }
    *cached_index = -1;
    return false;
}

API::FName make_fname(const wchar_t* name) {
    API::FName fn{name, API::FName::EFindName::Add};
    if (fn.comparison_index != 0) return fn;

    fn = API::FName{name, API::FName::EFindName::Find};
    if (fn.comparison_index != 0) return fn;

    static API::UObject* ksl = nullptr;
    static bool tried = false;
    if (!tried) {
        tried = true;
        if (auto* cls = API::get()->find_uobject<API::UClass>(
                L"Class /Script/Engine.KismetStringLibrary")) {
            ksl = cls->get_class_default_object();
        }
        API::get()->log_info("[Halo-CampE-UEVR] FName fallback via KismetStringLibrary: %s",
                             ksl != nullptr ? "available" : "NOT FOUND");
    }
    if (ksl == nullptr) return API::FName{};

    alignas(16) uint8_t p[RIG_PARAM_BUF] = {0};
    const int32_t len = (int32_t)wcslen(name) + 1;
    *reinterpret_cast<const wchar_t**>(p)   = name;
    *reinterpret_cast<int32_t*>(p + 8)      = len;
    *reinterpret_cast<int32_t*>(p + 12)     = len;
    ksl->call_function(L"Conv_StringToName", p);

    API::FName out{};
    memcpy(&out, p + 16, sizeof(int32_t) * 2);
    return out;
}

bool fname_is_sane(const std::string& n) {
    if (n.empty() || n.size() > 64) return false;
    for (unsigned char c : n) if (c < 0x20 || c > 0x7E) return false;
    return n != "None";
}

} // namespace halo
