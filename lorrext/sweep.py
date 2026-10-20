"""Run a sweep described by a JSON spec (written by the UI's New sweep screen).

    python -m lorrext.sweep runs/NAME/sweep.json [--workers 5]
"""

from __future__ import annotations

import argparse
import json
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path

from .core import Setting, run


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("spec", type=Path)
    ap.add_argument("--workers", type=int, default=5)
    a = ap.parse_args()
    spec = json.loads(a.spec.read_text())
    base = Setting(**spec["setting"])
    jobs = [(sv, replace(base, rate=r, seed=sd)) for sd in range(spec["seeds"]) for r in spec["rates"] for sv in spec["solvers"]]
    print(f"{len(jobs)} runs", flush=True)
    with ThreadPoolExecutor(a.workers) as ex:
        for i, r in enumerate(ex.map(lambda j: run(j[0], j[1], a.spec.parent, screen=spec.get("screen", 3)), jobs), 1):
            print(f"[{i}/{len(jobs)}] {r['solver']} rate={r['rate']} seed={r['seed']} rc={r['returncode']} "
                  f"thr={r.get('throughput', float('nan')):.3f} p99={r.get('lat_p99', float('nan')):.0f}", flush=True)


if __name__ == "__main__":
    main()
