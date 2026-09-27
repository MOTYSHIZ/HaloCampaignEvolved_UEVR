"""Fail if any fork setting key is also parsed by the upstream mod.

A fork key that reuses an upstream name is silently captured by whichever parser sees it first,
so one value ends up meaning two things. That happened once: the fork's reload hold feature was
named `reloadhold`, which the upstream parser reads as reload_hold_ms (the tap-vs-hold threshold,
clamped to 60..2000 ms). `reloadhold=1` turned every reload press into a hold, the press passed
to the game, and manual reload never started.

Upstream parsers: src/*.cpp and src/palettearm/*.cpp. Fork parsers: src/core/** and src/features/**.
OBSERVERS lists keys the fork only records the layer of (it never sets a value), which is allowed.
Run from the repository root: python scripts/check-key-collisions.py
"""
import os, re, sys

OBSERVERS = {"armhidemode", "armhidebone", "rig"}   # core/registry/Features.cpp layer tracking only
KEY = re.compile(r'_stricmp\(\s*key\s*,\s*"([A-Za-z0-9_]+)"\s*\)\s*==\s*0')

def scan(paths):
    found = {}
    for path in paths:
        with open(path, encoding="utf-8", errors="ignore") as fh:
            for n, line in enumerate(fh, 1):
                for m in KEY.finditer(line):
                    found.setdefault(m.group(1).lower(), f"{path}:{n}")
    return found

def cpp_in(folder, recurse):
    if not os.path.isdir(folder): return []
    if not recurse:
        return [os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(".cpp")]
    return [os.path.join(r, f) for r, _, fs in os.walk(folder) for f in fs if f.endswith(".cpp")]

upstream = scan(cpp_in("src", False) + cpp_in(os.path.join("src", "palettearm"), False))
fork = scan(cpp_in(os.path.join("src", "core"), True) + cpp_in(os.path.join("src", "features"), True))
clash = sorted(k for k in set(upstream) & set(fork) if k not in OBSERVERS)
print(f"upstream keys {len(upstream)}, fork keys {len(fork)}, collisions {len(clash)}")
for k in clash:
    print(f"  COLLISION {k}: upstream {upstream[k]}  fork {fork[k]}")
sys.exit(1 if clash else 0)
