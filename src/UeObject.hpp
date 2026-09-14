// Low-level UE object and name helpers, shared by every feature in the plugin.
//
// This is the layer between "the UEVR API" and the game-specific systems. It has no opinion about
// aim, rigs or reticules -- if something here starts caring about one of those, it belongs in that
// feature's module instead.
//
// The one thing worth understanding before using it: UE recycles object slots. A raw
// API::UObject* can silently come to point at a DIFFERENT object of a different class. That is what
// TrackedObject exists to prevent -- always prefer it over a bare pointer for anything held across
// frames.

#pragma once

// API.hpp, NOT Plugin.hpp. Plugin.hpp *defines* the plugin entry points (DllMain,
// uevr_plugin_initialize, uevr_plugin_required_version), so including it from a second translation
// unit produces duplicate-symbol link errors. Only Plugin.cpp may include Plugin.hpp.
#include "uevr/API.hpp"

#include <cstdint>
#include <string>

namespace halo {

// Scratch buffer for UFunction parameter blocks. 512 bytes is comfortably larger than any parameter
// struct called here, and over-allocating costs nothing on the stack.
// (Named for the rig, where it was first needed; it is general.)
constexpr size_t RIG_PARAM_BUF = 512;

// Lossy wide->narrow, for logging only. Non-ASCII becomes '?' -- never use it to build a name that
// will be fed back into the engine.
std::string narrow(const std::wstring& w);

// Class name of an object, or L"" if anything on the path is null. Null-safe at every step because
// this runs against Blam-backed objects that can be mid-teardown.
std::wstring class_name_of(uevr::API::UObject* obj);

// Resolve (or create) an FName.
//
// Add, then Find, then -- if both fail -- KismetStringLibrary::Conv_StringToName as a last resort.
// The fallback exists because FName construction does not reliably intern on this build, and a
// zero comparison_index silently makes every subsequent lookup miss.
uevr::API::FName make_fname(const wchar_t* name);

// Reject FName strings that cannot be real: empty, over-long, non-printable, or "None". Cheap guard
// against reading a recycled or partially-constructed object's name.
bool fname_is_sane(const std::string& n);

// IS THIS POINTER A LIVE UObject RIGHT NOW -- in O(1), with no walk of the object array.
//
// For a pointer read out of memory we do not own (a raw offset into a game struct), IsBadReadPtr
// proves only that the bytes are mapped. A freed object's block goes to the next allocation, so a
// stale pointer stays readable, and whatever now sits at its +0x10 is handed to UEVR as a UClass*
// whose FName is then read. MEASURED 2026-09-08 (42 aborted ticks): FName::ToString faulting on
// 0x40400018 = &((UObject*)0x40400000)->NamePrivate, and 0x40400000 is the float 3.0f -- a
// recycled block, not an object.
//
// The object array is the identity: a live object's InternalIndex names the slot that holds it.
// Garbage at +0xC is out of range or names a slot holding something else. One indexed compare,
// so it is safe on a per-element, per-tick path where uobject_live()'s array walk is not.
// Fails CLOSED (false) when the array is unavailable: the caller is about to dereference.
bool uobject_slot_valid(const uevr::API::UObject* p);

// A pointer PLUS its slot in the global object array, so a recycled slot can be detected instead of
// silently followed. Prefer this to a raw pointer for anything kept across frames.
struct TrackedObject {
    uevr::API::UObject* ptr = nullptr;
    int32_t             index = -1;

    // Adopt a pointer, locating its slot in the object array (one O(n) scan at acquisition).
    void set(uevr::API::UObject* p) {
        ptr = p; index = -1;
        if (p == nullptr) return;
        auto* arr = uevr::API::get()->get_uobject_array();
        if (arr == nullptr) { ptr = nullptr; return; }
        const int32_t n = arr->get_object_count();
        for (int32_t i = 0; i < n; ++i) {
            if (arr->get_object(i) == p) { index = i; return; }
        }
        ptr = nullptr;   // not in the array: never trust it
    }

    // Adopt a pointer whose array slot is already known (objects found by scanning).
    void set_at(uevr::API::UObject* p, int32_t i) { ptr = p; index = i; }

    // The pointer if its slot still holds it, else nullptr (and the handle resets).
    uevr::API::UObject* get() {
        if (ptr == nullptr || index < 0) return nullptr;
        auto* arr = uevr::API::get()->get_uobject_array();
        if (arr == nullptr || index >= arr->get_object_count() || arr->get_object(index) != ptr) {
            ptr = nullptr; index = -1;
            return nullptr;
        }
        return ptr;
    }

    // get() plus a class-name check, for slots that can be recycled at the same address.
    uevr::API::UObject* get_checked(const wchar_t* class_substr) {
        auto* p = get();
        if (p == nullptr) return nullptr;
        if (class_name_of(p).find(class_substr) == std::wstring::npos) {
            ptr = nullptr; index = -1;
            return nullptr;
        }
        return p;
    }

    void reset() { ptr = nullptr; index = -1; }
    bool empty() const { return ptr == nullptr; }
};

// IS THIS RAW POINTER STILL THE OBJECT WE ADOPTED?
//
// TrackedObject is the right answer wherever a handle can be stored, and 40 sites use it. This
// is the same check for the handful of places that hold a RAW pointer because it is read from
// dozens of call sites and threading a handle through all of them would be the larger change.
//
// WHY IT IS THE ONLY CHECK THAT WORKS. Measured 2026-09-04: a level teardown frees a component,
// its memory is handed to the next allocation, and the stale pointer then passes IsBadReadPtr
// AND a reflected class-name test -- because something that really is a component now lives
// there. Handing it to UEVR faults inside UEVR (0xC0000005 reading a live-looking heap
// address), so the crash never names us. The object array slot is the identity: if it no
// longer holds our pointer, the object we adopted is gone whatever occupies that address now.
//
// COST. Resolving the index is one walk of the ~296k object array, so it happens ONLY when the
// pointer changes; steady-state verification is a single indexed compare. A per-tick walk is
// the exact cost this project was bitten by in the find_uobject miss, and must not come back.
//
// Fails OPEN when the array is unavailable: unverifiable is not the same as known-dead.
bool uobject_live(uevr::API::UObject* p, int32_t* cached_index);

} // namespace halo
