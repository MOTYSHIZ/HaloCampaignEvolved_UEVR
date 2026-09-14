#pragma once

// CFG KEYS OWNED BY CORE SERVICES. Tuning read by core code belongs to core, so no core service reads a
// key a feature parses. Called by features_parse_key before any feature's parse slot; every key here is
// parsed nowhere else, so the order does not change a result.

namespace halo {

bool core_parse_key(const char* key, const char* val, double v);

} // namespace halo
