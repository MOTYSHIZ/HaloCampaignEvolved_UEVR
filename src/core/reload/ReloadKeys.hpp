#pragma once

// The reload engine's cfg keys (the reload family's subkeys; core/reload reads them).

namespace halo {

bool reload_engine_parse_key(const char* key, const char* val, double v);

} // namespace halo
