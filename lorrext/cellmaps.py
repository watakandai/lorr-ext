"""Per-cell service-time and delay maps.

A *service map* says how long work takes at each cell ("cutting onions at
(10, 12) takes lognormal(mean 12, cv 0.4) ticks"). A *delay map* says how
likely a robot standing on a cell is to get stuck for a while, and for how long.
Both are what you would estimate from past data for a real site.

Specs accepted by Setting.service / Setting.delay:

    "none"                              nothing (deterministic; the default)
    preset name                         see SERVICE_PRESETS / DELAY_PRESETS
    "all:<dist>"                        same distribution on every cell, e.g.
                                        service "all:lognormal:20:0.5",
                                        delay   "all:0.02:gamma:3:0.5" (p, then dist)
    "char:<C>:<dist>"                   service on every cell marked C in the map file,
                                        e.g. "char:E:normal:8:0.3" = picking time at the
                                        stations of a warehouse map (needs the map's chars)
    path/to/map.json                    explicit map in the instance format:
        {"default": {...} | null,
         "cells": [{"name": "onions", "cells": [[10, 12], ...],
                    "dist": "lognormal", "mean": 12, "cv": 0.4,     # service
                    "p": 0.05}, ...]}                               # delay only
Distribution strings: "lo-hi" (uniform interval) or "dist:mean[:cv[:min[:max]]]"
with dist in uniform, fixed, exponential, lognormal, gamma, normal.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

DISTS = ("uniform", "fixed", "exponential", "lognormal", "gamma", "normal")


def parse_dist(spec: str) -> dict:
    """'lo-hi' or 'dist:mean[:cv[:min[:max]]]' -> dict for the C++ side."""
    if ":" not in spec:
        lo, hi = (int(x) for x in spec.split("-"))
        return {"dist": "uniform", "lo": lo, "hi": hi}
    name, *nums = spec.split(":")
    if name not in DISTS:
        raise ValueError(f"unknown distribution {name!r}; choose from {DISTS}")
    d = {"dist": name, **{k: float(v) for k, v in zip(["mean", "cv", "min", "max"], nums)}}
    if name == "exponential":
        d["cv"] = 1.0
    return d


def dist_mean(d: dict) -> float:
    return (d["lo"] + d["hi"]) / 2 if d["dist"] == "uniform" else d.get("mean", 0.0)


# ----------------------------------------------------------------- map geometry
def read_free(map_file: Path) -> np.ndarray:
    lines = Path(map_file).read_text().splitlines()
    h, w = int(lines[1].split()[1]), int(lines[2].split()[1])
    return np.array([[ch in ".GSE" for ch in row[:w]] for row in lines[4:4 + h]])


def free_degree(free: np.ndarray) -> np.ndarray:
    """Number of free 4-neighbours of each cell (corridors / dead ends have <= 2)."""
    p = np.pad(free, 1)
    return (p[:-2, 1:-1].astype(int) + p[2:, 1:-1] + p[1:-1, :-2] + p[1:-1, 2:]) * free


# ----------------------------------------------------------------- presets
def _kitchen(free: np.ndarray, seed: int) -> dict:
    """Four work zones (quadrants), each with its own service-time distribution."""
    h, w = free.shape
    zones = [("pick ingredients", "lognormal:3:0.3", (0, h // 2, 0, w // 2)),
             ("cut onions", "lognormal:12:0.4", (0, h // 2, w // 2, w)),
             ("cook", "gamma:20:0.5", (h // 2, h, 0, w // 2)),
             ("plate & hand over", "normal:5:0.2:1", (h // 2, h, w // 2, w))]
    groups = []
    for name, spec, (r0, r1, c0, c1) in zones:
        cells = [[int(r), int(c)] for r in range(r0, r1) for c in range(c0, c1) if free[r, c]]
        groups.append({"name": name, "cells": cells, **parse_dist(spec)})
    return {"default": None, "cells": groups}


def _stations(free: np.ndarray, seed: int) -> dict:
    """10% of cells are work stations with slow, variable service; elsewhere instant."""
    rng = np.random.default_rng(seed)
    cells = np.argwhere(free)
    pick = cells[rng.random(len(cells)) < 0.10]
    kinds = [("cut onions", "lognormal:12:0.4"), ("cook", "gamma:30:0.5"), ("wash", "normal:8:0.2:1")]
    groups = [{"name": n, "cells": [[int(r), int(c)] for r, c in pick[i::len(kinds)]], **parse_dist(s)}
              for i, (n, s) in enumerate(kinds)]
    return {"default": None, "cells": groups}


def _kit_like(free: np.ndarray, seed: int) -> dict:
    """The 2026 kit's own example: 1% chance per tick anywhere, 1-4 ticks."""
    return {"default": {"p": 0.01, **parse_dist("1-4")}, "cells": []}


def _congestion(free: np.ndarray, seed: int) -> dict:
    """Narrow cells (<= 2 free neighbours) are where robots get stuck."""
    deg = free_degree(free)
    narrow = [[int(r), int(c)] for r, c in np.argwhere(free & (deg <= 2))]
    return {"default": {"p": 0.002, **parse_dist("1-2")},
            "cells": [{"name": "narrow passage", "cells": narrow, "p": 0.05, **parse_dist("lognormal:4:0.6:1")}]}


SERVICE_PRESETS = {"kitchen": _kitchen, "stations": _stations}
DELAY_PRESETS = {"kit_like": _kit_like, "congestion": _congestion}


def read_chars(map_file: Path) -> np.ndarray:
    lines = Path(map_file).read_text().splitlines()
    h, w = int(lines[1].split()[1]), int(lines[2].split()[1])
    return np.array([list(row[:w]) for row in lines[4:4 + h]])


def build(spec: str, kind: str, free: np.ndarray, seed: int = 0, chars: np.ndarray | None = None) -> dict | None:
    """Spec -> map dict in the instance format (None = feature off)."""
    if spec in ("", "none"):
        return None
    if spec.startswith("char:"):
        _, ch, dist = spec.split(":", 2)
        if chars is None:
            raise ValueError(f"{spec!r} needs the map's characters")
        cells = [[int(r), int(c)] for r, c in np.argwhere(chars == ch)]
        if kind == "delay":
            p, dist = dist.split(":", 1)
            return {"default": None, "cells": [{"name": f"cells {ch}", "cells": cells, "p": float(p), **parse_dist(dist)}]}
        return {"default": None, "cells": [{"name": f"cells {ch}", "cells": cells, **parse_dist(dist)}]}
    presets = SERVICE_PRESETS if kind == "service" else DELAY_PRESETS
    if spec in presets:
        return presets[spec](free, seed)
    if spec.startswith("all:"):
        rest = spec[4:]
        if kind == "delay":
            p, dist = rest.split(":", 1)
            return {"default": {"p": float(p), **parse_dist(dist)}, "cells": []}
        return {"default": parse_dist(rest), "cells": []}
    if spec.endswith(".json"):
        return json.loads(Path(spec).read_text())
    raise ValueError(f"bad {kind} spec {spec!r}: use none, a preset {sorted(presets)}, 'all:...', or a .json file")


def per_cell(m: dict | None, free: np.ndarray, field: str = "mean") -> np.ndarray:
    """Grid of a per-cell quantity for plots/metrics: 'mean' (expected duration) or 'p'."""
    out = np.full(free.shape, np.nan)
    if m is None:
        out[free] = 0.0
        return out
    if m.get("default"):
        out[free] = dist_mean(m["default"]) if field == "mean" else m["default"].get("p", 0.0)
    else:
        out[free] = 0.0
    for g in m.get("cells", []):
        v = dist_mean(g) if field == "mean" else g.get("p", 0.0)
        for r, c in g["cells"]:
            out[r, c] = v
    return out
