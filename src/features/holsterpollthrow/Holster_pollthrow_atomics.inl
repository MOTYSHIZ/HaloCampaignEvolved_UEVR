// holsterpollthrow (fork feature, Experimental): the poll-rate throw verdict and the Blam-unit hand and swing publishes.
// Textual fragment, included by Holster.cpp in its anonymous namespace, after the torso state. Moved verbatim; not compiled on its own.
// ---- POLL-RATE THROW RELEASE (doctrine in Config.hpp). The tick's standing verdict, consumed by
// the XInput hook. Mask 0 = disarmed (no grenade, hand in a pouch = put-back territory, feature
// off). The hold angles are the tick's peak-direction math, republished every tick so the hook
// only ever copies numbers.
std::atomic<unsigned short> g_pollthrow_mask{0};
std::atomic<float> g_pollthrow_hy{0.0f}, g_pollthrow_hp{0.0f};
std::atomic<bool>  g_pollthrow_hold{false};
std::atomic<bool>  g_pollthrow_fired{false};

// The carrier hand's position in BLAM units, for the grenade-at-hand spawn-origin experiment
// (grenhand, doctrine in Config.hpp). Published per tick while a grenade is armed; the last value
// deliberately survives the release, because the spawn lands ~42 ms after it.
std::atomic<float> g_hand_blam_x{0.0f}, g_hand_blam_y{0.0f}, g_hand_blam_z{0.0f};
std::atomic<bool>  g_hand_blam_valid{false};
// The swing's peak direction in BLAM units (normalized), for the instant-release velocity.
std::atomic<float> g_throw_blam_x{0.0f}, g_throw_blam_y{1.0f}, g_throw_blam_z{0.0f};
std::atomic<bool>  g_throw_blam_valid{false};
