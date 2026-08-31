#include "TwoHand.hpp"

#include <algorithm>

namespace halo::palettearm {

void TwoHandHold::reset() {
    m_state = {};
    m_last_forward = {};
    m_has_last_forward = false;
}

TwoHandState TwoHandHold::update(const TwoHandInput& input, const TwoHandTuning& tuning) {
    const bool was_latched = m_state.latched;

    bool in_zone = false;
    bool latched = false;
    m_state.measured = false;   // set true only if the geometry below is actually computed

    if (input.gameplay_active && input.support_tracked && valid_basis(input.aim_basis)) {
        // Is the support hand on the barrel? Project the hand-to-hand vector onto the aim ray and
        // measure how far along it lies and how far off the line it sits.
        // Everything here is in the CALLER'S frame and the caller's unit; only the comparison
        // against the zone is in metres. Keeping the projection in native units and converting
        // once at the end is what stops the scale factor being applied twice (it was, once).
        const Vec3  aim_forward = normalized(input.aim_basis.forward);
        const Vec3  hand_line   = input.support_grip_position - input.aim_grip_position;
        const float along_units = dot(hand_line, aim_forward);
        const Vec3  perpendicular = hand_line - aim_forward * along_units;
        const float along   = along_units * tuning.units_to_metres;
        const float lateral = length(perpendicular) * tuning.units_to_metres;

        m_state.along_m   = along;
        m_state.lateral_m = lateral;
        m_state.measured  = std::isfinite(along) && std::isfinite(lateral);

        in_zone = m_state.measured &&
                  along > tuning.zone_min_along_m &&
                  along < tuning.zone_max_along_m &&
                  lateral < tuning.zone_radius_m;

        // Acquisition needs the zone; RETENTION needs only the button. See the header.
        latched = input.support_grip_held && (was_latched || in_zone);

        // Remember the line while it is good, so a tracking drop mid-hold can ease out along it
        // instead of snapping the weapon back to one-handed aim.
        if (latched) {
            const Vec3 forward = normalized(hand_line);
            if (length_squared(forward) >= 0.8f) {
                m_last_forward = forward;
                m_has_last_forward = true;
            }
        }
    }

    m_state.in_zone       = in_zone;
    m_state.latched       = latched;
    m_state.latch_changed = (latched != was_latched);
    if (!latched && m_state.blend <= 0.0f) m_has_last_forward = false;

    // Ease the influence rather than switching it. delta is clamped so a hitch or a debugger pause
    // cannot jump the blend across in one frame -- the ease exists to hide exactly that.
    const float target = latched ? 1.0f : 0.0f;
    const float step   = (tuning.blend_seconds > 1.0e-4f)
                       ? std::clamp(input.delta_seconds, 0.0f, 0.25f) / tuning.blend_seconds
                       : 1.0f;
    m_state.blend = (m_state.blend < target) ? std::min(target, m_state.blend + step)
                                             : std::max(target, m_state.blend - step);
    return m_state;
}

Mat3 TwoHandHold::effective_basis(const Mat3& one_hand_basis,
                                  const Vec3& aim_grip_position,
                                  const Vec3& support_grip_position,
                                  bool support_tracked,
                                  const TwoHandTuning& tuning) const {
    if (m_state.blend <= 0.0f || !valid_basis(one_hand_basis)) return one_hand_basis;

    Vec3 two_hand_forward{};
    if (support_tracked) {
        two_hand_forward = normalized(support_grip_position - aim_grip_position);
        if (length_squared(two_hand_forward) < 0.8f) return one_hand_basis;
    } else {
        if (!m_has_last_forward) return one_hand_basis;
        two_hand_forward = m_last_forward;
        if (!finite(two_hand_forward) || length_squared(two_hand_forward) < 0.5f) {
            return one_hand_basis;
        }
    }

    // Fade in across the agreement band instead of hard-gating. Below the minimum the aim is
    // exactly one-handed; the smoothstep keeps it continuous when a latched support hand crosses
    // the boundary, which is the ~70-degree snap the band exists to remove.
    //
    // The floor looks redundant with the smoothstep -- it is not, and a mutation run proved it.
    // Falling through with a zero weight still rebuilds the basis from cross(up, forward), which
    // re-orthonormalises an input valid_basis() deliberately tolerates as slightly drifted. This
    // early-out returns the caller's basis BIT-IDENTICAL, which is what "exactly one-handed" has
    // to mean for something the player's aim rides on.
    const float agreement = dot(two_hand_forward, one_hand_basis.forward);
    if (!std::isfinite(agreement) || agreement < tuning.minimum_agreement) return one_hand_basis;
    const float weight =
        m_state.blend * smoothstep(tuning.minimum_agreement, tuning.full_agreement, agreement);

    const Vec3 blended_forward =
        normalized(one_hand_basis.forward + (two_hand_forward - one_hand_basis.forward) * weight);
    // Roll stays with the aim hand: the support hand sets where the gun POINTS, not how it is
    // rotated about its own barrel. Deriving left from the aim hand's up is what keeps that true.
    Vec3 blended_left = cross(one_hand_basis.up, blended_forward);
    if (!finite(blended_forward) || length_squared(blended_left) < 1.0e-6f) return one_hand_basis;
    blended_left = normalized(blended_left);

    const Mat3 result{blended_forward, blended_left,
                      normalized(cross(blended_forward, blended_left))};
    return valid_basis(result) ? result : one_hand_basis;
}

} // namespace halo::palettearm
