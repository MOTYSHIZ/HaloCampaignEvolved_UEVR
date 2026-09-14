// THE FORK'S CFG VALUE TYPES, included by the author's Config.hpp inside namespace halo, right after
// kMaxWeaponAdjust (the place the fork had them). Declarations only: no includes, no namespace.

// One radar blip colour, keyed on the contact's +0x177 BODY-CLASS byte (0x0D Grunt, 0x0E
// Elite-or-marine, others unsurveyed). Species colouring, not faction: until the faction field is
// found, anything Elite-sized shares a colour with a marine.
struct BlipColor {
    uint32_t cls = 0;               // the SPECIES ID (object's leading dword) this paints
    float    r = 1.0f, g = 1.0f, b = 1.0f;
};
constexpr int kMaxBlipColor = 12;

// A colour keyed on the SPECIES NAME -- the Unreal class of the actor standing where the contact
// is. Unlike a tag id this is the same on every level, so one entry holds for the campaign.
struct BlipName {
    char  match[64] = "";           // substring of the actor class, e.g. "Grunt"
    float r = 1.0f, g = 1.0f, b = 1.0f;
};
constexpr int kMaxBlipName = 16;

// One physical-scope entry (Scope.hpp): weapon key, capture FOV (deg), lens offset (cm) and
// rotation (deg, pitch/yaw/roll) in the FP weapon root's frame, lens scale (plane is 100 cm).
struct ScopeCfg {
    char  key[64] = {0};
    float fov = 12.0f;
    float pos[3] = {0.0f, -15.0f, 8.0f};
    float rot[3] = {0.0f, 0.0f, 0.0f};
    float size = 0.04f;
};
