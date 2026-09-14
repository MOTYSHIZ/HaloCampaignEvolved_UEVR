// palettewpn (fork feature, Experimental): the socket world position and rotation readbacks the palette instruments use.
// Textual fragment, included by Rig.cpp at the end of namespace halo. Moved verbatim; not compiled on its own.
// The socket's WORLD position, straight from the posed skeleton. Same read derive_pivot does, minus
// the component-relative step -- kept separate rather than folded in because the two answer
// different questions and derive_pivot's fail-closed length check is about pivot sanity, not about
// whether a readback succeeded.
bool rig_socket_world(API::UObject* rig, const wchar_t* socket, Vec3* out) {
    if (rig == nullptr) return false;
    Vec3 sock{};
    if (!call_socket_location(rig, socket, &sock)) return false;
    if (!std::isfinite(sock.x) || !std::isfinite(sock.y) || !std::isfinite(sock.z)) return false;
    *out = sock;
    return true;
}

// GetSocketRotation(FName) -> FRotator (pitch, yaw, roll as doubles), same layout rules as the
// location call: FName at 0, return at offset 8. The socket's WORLD rotation from the posed
// skeleton -- the gun's pointing direction as rendered, not as written. Position alone proved the
// bone lands where we put it; this is what makes the barrel-vs-aim-ray angle a logged number.
bool rig_socket_world_rot(API::UObject* rig, const wchar_t* socket, Vec3* out_pyr) {
    if (rig == nullptr || out_pyr == nullptr) return false;
    alignas(16) uint8_t params[RIG_PARAM_BUF] = {0};
    API::FName name = make_fname(socket);
    memcpy(params, &name, sizeof(int32_t) * 2);
    rig->call_function(L"GetSocketRotation", params);
    auto* d = reinterpret_cast<double*>(params + 8);
    if (!std::isfinite(d[0]) || !std::isfinite(d[1]) || !std::isfinite(d[2])) return false;
    *out_pyr = Vec3{(float)d[0], (float)d[1], (float)d[2]};
    return true;
}
