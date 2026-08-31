// XrLayerAttach -- finding the OpenXR entry points that UEVR ACTUALLY calls.
//
// ============================================================================================
// THE PROBLEM THIS EXISTS FOR (measured 2026-08-23)
// ============================================================================================
// UEVR statically links the OpenXR loader into UEVRBackend.dll. `dumpbin /imports` on it shows no
// openxr_loader.dll import at all. Two consequences, and the second is worse than the first:
//
//   1. Hooking openxr_loader.dll!xrEndFrame intercepts nothing. UEVR never calls it. The hook
//      installs cleanly and runs zero times -- which is exactly how this was found, from a state
//      line that said Pending forever.
//
//   2. LESS OBVIOUS AND MORE DANGEROUS: neither can the OTHER entry points come from that DLL.
//      The XrSession UEVR hands us through the plugin API belongs to the loader instance inside
//      UEVRBackend.dll. Passing it to openxr_loader.dll!xrCreateSwapchain would be handing a
//      handle to a DIFFERENT loader's dispatch table. That is not a subtle mismatch; it is an
//      unrelated pointer being dereferenced as a dispatch table.
//
// openxr_loader.dll IS loaded in the Halo process -- Virtual Desktop's streamer pulls it in -- so
// GetModuleHandleW finds it, GetProcAddress returns real functions, and every check passes. That
// is precisely what made the first attempt look correct.
//
// ============================================================================================
// HOW WE RESOLVE INSTEAD
// ============================================================================================
// UEVR ships UEVRBackend.pdb beside UEVRBackend.dll, and it contains the loader's symbols
// (verified: xrEndFrame, xrCreateSwapchain, xrAcquireSwapchainImage, xrGetInstanceProcAddr are all
// present). So the address is looked up BY NAME through dbghelp -- authoritative, and it tracks
// UEVR pin bumps automatically because a PDB always matches its own DLL. There is no byte
// signature here and no recorded offset, so there is nothing to re-measure when the pin moves.
//
// DEV-ONLY, per DevTools.hpp. Not because it is dangerous, but because it depends on a PDB that
// only exists in a UEVR checkout: players have no UEVRBackend.pdb, so this could never be the
// shipping attachment. The shipping route is an OpenXR API layer, which the statically-linked
// loader loads through the normal implicit-layer chain and which needs no symbol lookup at all.
// This module is the PROVING HARNESS that lets the swapchain, blit, pose math and colour override
// be tested in a headset now, rather than after that layer is built.
//
// THAT LAYER NOW EXISTS. `apilayer\` builds it, `src\XrLayerBridge.hpp` is the plugin's side of it,
// and `src\XrLayerAbi.h` is the contract between the two -- see the ApiLayer tier below. This module
// keeps its job regardless: it is still the attachment that works in a checkout with nothing
// registered, and it is still the fallback for a player who has not enabled the layer.
//
// KNOWN RESIDUAL RISK, deliberately not designed around: if UEVR is built with LTCG the loader's
// xrEndFrame may be INLINED into OpenXR::end_frame, in which case the symbol still resolves, the
// hook still installs, and it is still never called. That is not worth guessing about in advance --
// XrLayer's watchdog reports it within ~3 s of live frames, which is faster than reasoning about it.

#pragma once

#include "DevTools.hpp"

#include <cstddef>

namespace halo {

// Where an entry point came from. Reported once, because an address nobody can name in the log is
// an address nobody can check after an update.
// THE LADDER, IN PREFERENCE ORDER -- WHICH IS NOT THE DECLARATION ORDER. `ApiLayer` is appended
// last only so that no existing value moves; it RANKS FIRST. Read the comments, not the numbers.
enum class XrAttachTier {
    None = 0,
    BackendPdb,     // symbol lookup in UEVRBackend.pdb -- the one that works with UEVR
    LoaderExport,   // openxr_loader.dll export -- correct for a host that DYNAMICALLY links the
                    // loader, wrong for UEVR. Kept as a rung rather than deleted: it costs nothing,
                    // it is the right answer for some other host, and a fallback that was removed
                    // because it did not fit today is a fallback nobody can re-enable tomorrow.

    // ---- THE SHIPPING RUNG. Prefer this over every tier above it when it is available. ----
    //
    // The entry points are handed to us by XrApiLayer_HALOVR_reticule.dll -- our own OpenXR API
    // layer, which the loader has loaded into this process through the ordinary layer chain.
    //
    // It answers the objection this whole file is built around. The PDB tier is exact and correct
    // and CANNOT REACH PLAYERS: UEVRBackend.pdb only exists in a UEVR checkout, so on a player
    // install the resolve fails and the feature latches off silently while shipping enabled. A
    // layer needs no PDB, no signature, no recorded address and no hook at all -- the loader hands
    // it xrEndFrame, and it does so even though UEVR statically links its loader, because the
    // loader still walks the API-layer registry at instance creation.
    //
    // It is also the only rung with no rot surface. There is nothing here to re-measure when the
    // UEVR pin moves, when the game patches, or when a different store's binary is used.
    //
    // NOTE THAT THIS FILE DOES NOT IMPLEMENT IT, and that is deliberate rather than an omission.
    // The tier is owned by XrLayerBridge.hpp/.cpp, because reaching it is not a symbol lookup at
    // all -- it is GetModuleHandleW plus one GetProcAddress plus an ABI handshake, which has
    // nothing in common with the dbghelp machinery here and no reason to load a 142 MB PDB to find
    // out. xrattach_resolve() therefore never returns this value; ask xrbridge_available() first
    // and fall through to xrattach_resolve() only when the layer is absent.
    ApiLayer,
};

// Resolve one OpenXR entry point by name, trying the tiers in order.
//
// Returns nullptr if every tier failed, having logged why. `out_tier` (optional) reports which rung
// won so the caller can say so once.
//
// SAFE TO CALL FROM ANY THREAD, but the FIRST call is expensive: it loads a ~142 MB PDB. Call it
// off the game thread -- xrattach_begin_async() below does exactly that.
void* xrattach_resolve(const char* symbol, XrAttachTier* out_tier);

// Kick off symbol loading on a detached worker, for the same reason MemScan runs on one: the PDB
// load takes far longer than a frame, and doing it on the game thread would not be a stutter but a
// fault. One shot; safe to call repeatedly.
void xrattach_begin_async();

// True once the worker has finished (successfully or not). Until then xrattach_resolve() would
// block, so callers should wait for this rather than ask early.
bool xrattach_ready();

// Human-readable one-liner for the log: which tier won and where the module was found.
const char* xrattach_status();

}   // namespace halo
