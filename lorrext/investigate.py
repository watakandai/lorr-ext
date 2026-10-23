"""Root-cause experiments (stage 4 of docs/research). Each subcommand tests one hypothesis.

    python -m lorrext.investigate e1     # 2026 kit: is the gap to the 2024 default plan latency?

Runs go to runs/investigate/<name>/ and are cached like the benchmark's.
"""

from __future__ import annotations

import argparse
import json
from contextlib import contextmanager
from pathlib import Path

from . import core
from .core import Setting

OUT = Path("runs/investigate")


@contextmanager
def timing(tick_ms: int, window_ms: int):
    """Temporarily run the 2026 kit with another tick length / planner window."""
    old = core.TICK_MS, core.WINDOW_MS
    core.TICK_MS, core.WINDOW_MS = tick_ms, window_ms
    try:
        yield
    finally:
        core.TICK_MS, core.WINDOW_MS = old


def e1(cpus: str = "12,13") -> list[dict]:
    """E1: with goals frozen for a planning window and plans arriving one window late, every
    goal change costs the 2026 default up to two windows. If that latency is the whole gap,
    a one-tick window with enough time for traffic guidance should reach the 2024 default."""
    s = Setting()  # competition setting: random map, 100 robots, pool
    rows = []
    for tick, window in [(30, 300), (100, 1000), (300, 900), (300, 300), (100, 100)]:
        with timing(tick, window):
            r = core.run("default2026", s, OUT / f"e1_t{tick}_w{window}", cpus=cpus, timeout=3600)
        r.update(tick_ms=tick, window_ms=window, window_ticks=window // tick)
        rows.append(r)
        print(f"tick {tick:3d} ms, window {window:4d} ms ({window // tick:2d} ticks): "
              f"throughput {r['throughput']:.3f}, p50 latency {r['lat_p50']:.0f}", flush=True)
    r = core.run("default2024", s, OUT / "e1_ref2024", cpus=cpus, timeout=3600)
    print(f"2024 default reference: throughput {r['throughput']:.3f}, p50 latency {r['lat_p50']:.0f}", flush=True)
    rows.append({**r, "tick_ms": None, "window_ms": None, "window_ticks": 1})
    (OUT / "e1.json").write_text(json.dumps(rows, indent=1))
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=["e1"])
    ap.add_argument("--cpus", default="12,13")
    a = ap.parse_args()
    {"e1": e1}[a.what](a.cpus)


if __name__ == "__main__":
    main()
