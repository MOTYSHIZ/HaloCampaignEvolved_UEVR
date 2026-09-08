# palettearm — Halo's own arms, posed in the Blam node palette

An **alternative arm driver**, ported from elliotttate's `HaloCampaignEvolved-UEVR`
(`main.cpp` @ `62ee34f`) with the author's permission. It is a *different solution to the same
problem* as `Rig.cpp` + `Arms.cpp` + `Hands.cpp`, not an addition to them — which is why it lives
in its own folder and why `ArmDriver.hpp` will only ever let one of the two run.

## The two routes, and why both exist

|  | **UE route** (`Rig`/`Arms`/`Hands`) | **Palette route** (this folder) |
|---|---|---|
| Layer | UE reflection, above the anim graph | Blam node palette, below UE entirely |
| Arms you see | game's FP mesh, driven as ONE unit; left chain bone-hidden | game's own arms, per-node |
| Hands | our spawned StaticMeshComponents on the controllers | the game's own hand nodes, fingers included |
| Blocked by | `SetBoneTransformByName` is **absent** on this build | needs a detour on the native FP weapon builder |
| Costs | a reflected call per tick per piece | memcpy-scale work inside a hook already on the frame |

`Hands.hpp` records the reasoning that produced the UE route: the game's bones cannot be posed
because UE does not expose the call. That is true *about UE*, and it is why our hands are spawned
spheres rather than the game's hands. It simply does not bind below UE — the palette is plain
memory, and the native builder writes it every frame whether UE exposes anything or not.

So neither route is "the right one" on paper. They fail differently, they feel different in a
headset, and the point of keeping both is to be able to A/B them there.

## Layout — and the rule that keeps this testable

    PaletteMath.hpp/.cpp   Vec3/Quat/Mat3/BlamMatrix4x3 and their algebra
    NodeMap.hpp            the 76-node first-person topology (which index is which knuckle)
    ArmSolve.hpp/.cpp      shoulder anchoring, two-bone IK, wrist placement, finger curl
    TwoHand.hpp/.cpp       the support-hand latch and the aim blend
    PaletteArm.hpp/.cpp    the module: config, the hook, the per-frame drive
    PaletteHook.hpp/.cpp   resolving and detouring the native FP weapon builder

**Everything above `PaletteArm.cpp` is pure.** No UEVR, no `Config.hpp`, no Windows, no globals —
inputs in, palette mutated, out. That is deliberate and it is load-bearing: it is the only reason
this can be verified at all without a headset. `Scripts\Verify-PaletteArm.ps1` compiles those four
translation units **out of tree with no include paths** and runs unit tests over the solvers, the
same trick and the same reasoning as `Scripts\Verify-AddrCascadeStandalone.ps1`. Run it after any
change in here; one convenient `#include "../Config.hpp"` would never fail the plugin build,
because every project header is already on that include path.

`PaletteArm.cpp` and `PaletteHook.cpp` are the impure edge and are not covered by that script.

## Do not duplicate the two-hand blend

`TwoHand.cpp` produces the aim basis that the *rendered* arms use. The same basis has to reach the
muzzle ray and the projectile spawn or the gun points somewhere the shots do not go — in his build
one function feeds the palette build, the marker hook and the projectile hook for exactly that
reason. When this is wired to our aim path, feed it from `two_hand_effective_basis()` rather than
recomputing the blend; two copies of a blend will diverge and the symptom is shots that miss where
the barrel is looking, which reads as an aim bug rather than a duplication bug.

## Attribution

The mechanisms are not originally his either, and his comments say so:

* **shoulder anchoring** — RoboquestVR's arm rig (yaw-only head frame, clavicle-rooted overreach)
* **two-handed hold** — Halo-MCC-VR's headset-tuned barrel grab. He widened its hard 0.35 agreement
  cutoff into a smoothstep band because the cutoff snaps the weapon ~70 degrees when a latched
  support hand crosses it. That change is his and it is worth keeping.
* **node tables in `NodeMap.hpp`** — read off the shipped Spartan assault-rifle skeleton with Baboon.

His repo carries **no LICENSE file**, so the permission we have is verbal. Get it in writing before
any of this ships, and credit all three projects in the README when it does.
