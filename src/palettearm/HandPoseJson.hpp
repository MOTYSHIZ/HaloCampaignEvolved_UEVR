#pragma once

// THE HAND POSE FILE (halo_vr_handposes.json). The pose table (ArmSolve.hpp: HandPose x
// kHandPoseCount) in JSON, laid out the way the struct is:
//
//   { "version": 1,
//     "poses": {
//       "fist": {
//         "index": { "curl": 1, "seg": [0, 0, 0], "rot": [[0, 0, 0], [0, 0, 0], [0, 0, 0]] },
//         ...
//         "thumb": { "curl": 1, "seg": [...], "rot": [...], "over": 0.35, "ext": 0, "out": 0.3 } },
//       ... } }
//
// Pose names: rest index fist thumbsup point pointdown ok. Fingers: index middle ring pinky thumb.
// Reading OVERLAYS the file onto a table: anything the file leaves out keeps the table's value, and
// a shorter array changes only the entries it has. Keys starting with '_' are notes and ignored.
//
// Pure: no engine, no file I/O, no project headers -- the plugin reads the file and hands the text
// in, and Scripts\Verify-PaletteArm.ps1 compiles this out of tree to prove it.

#include "ArmSolve.hpp"

#include <string>
#include <vector>

namespace halo::palettearm {

// The shipped poses, compiled in. Canonized 2026-09-18 from the user's headset tuning.
void default_hand_poses(HandPose out[kHandPoseCount]);

struct HandPoseJsonResult {
    bool ok{false};
    std::string error;                  // syntax / type error, with "line L col C"; table untouched
    std::vector<std::string> ignored;   // names that matched nothing ("poses.fsit", "ok.index.curll")
    int values{0};                      // numbers applied
};

// Parse `text` and overlay it onto `table`. On any error `table` is left exactly as it was.
HandPoseJsonResult hand_poses_from_json(const char* text, std::size_t len, HandPose table[kHandPoseCount]);

// The whole table as the file format, one finger per line. from_json(to_json(t)) == t.
std::string hand_poses_to_json(const HandPose table[kHandPoseCount]);

} // namespace halo::palettearm
