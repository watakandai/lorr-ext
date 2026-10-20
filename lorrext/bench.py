"""Benchmark grid runner.

    python -m lorrext.bench --out runs/bench --workers 5          # full grid
    python -m lorrext.bench --out runs/bench --only pool,poisson   # subsets
    python -m lorrext.bench --list                                 # print the plan

Each run's metrics row is cached in OUT/rows/, so an interrupted grid resumes.
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from .core import SOLVERS, Setting, run

RATES = [0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 1.0]
SOLVERS_2026 = [s for s, v in SOLVERS.items() if v["gen"] == "2026"]


def grid(steps: int = 1000) -> dict[str, list[tuple[str, Setting]]]:
    g: dict[str, list[tuple[str, Setting]]] = {}
    g["pool"] = [(sv, Setting(arrival="pool", seed=sd, steps=steps)) for sd in range(3) for sv in SOLVERS]
    g["poisson"] = [(sv, Setting(arrival="poisson", rate=r, seed=sd, steps=steps))
                    for sd in range(3) for r in RATES for sv in SOLVERS]
    g["mmpp"] = [(sv, Setting(arrival="mmpp", rate=r, seed=sd, steps=steps))
                 for sd in range(2) for r in RATES for sv in SOLVERS]
    g["kitchen"] = [(sv, Setting(arrival="poisson", rate=r, service="kitchen", seed=sd, steps=steps))
                    for sd in range(2) for r in RATES for sv in SOLVERS]
    g["hawkes"] = [(sv, Setting(arrival="hawkes", rate=r, seed=0, steps=steps)) for r in RATES for sv in SOLVERS]
    # the 2026 move-delay model exists only in the 2026 kit
    g["delay"] = [(sv, Setting(arrival=a, rate=r, p_delay=0.05, seed=0, steps=steps))
                  for a in ("poisson", "mmpp") for r in RATES for sv in SOLVERS_2026]
    # longer horizon at light load: ~30 scored tasks per 1000-tick run is too few for a 1% criterion
    g["lowload"] = [(sv, Setting(arrival="poisson", rate=r, seed=sd, steps=3 * steps))
                    for sd in range(2) for r in (0.05, 0.1, 0.2) for sv in SOLVERS]
    return g


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="runs/bench")
    ap.add_argument("--workers", type=int, default=5)
    ap.add_argument("--cores-per-run", type=int, default=2)
    ap.add_argument("--steps", type=int, default=1000)
    ap.add_argument("--only", default="", help="comma list of grid parts")
    ap.add_argument("--solvers", default="", help="comma list of solvers")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()

    g = grid(a.steps)
    parts = a.only.split(",") if a.only else [p for p in g if p != "lowload"]
    keep = set(a.solvers.split(",")) if a.solvers else set(SOLVERS)
    jobs = [j for p in parts for j in g[p] if j[0] in keep]
    # slow 2024 entries first so the tail of the grid is short runs
    jobs.sort(key=lambda j: SOLVERS[j[0]]["gen"] != "2024")
    print(f"{len(jobs)} runs: " + ", ".join(f"{p}={sum(j[0] in keep for j in g[p])}" for p in parts), flush=True)
    if a.list:
        return
    out = Path(a.out)
    t0, done, failed = time.time(), 0, 0
    slots: queue.Queue[str] = queue.Queue()
    for w in range(a.workers):  # dedicated cores per worker
        slots.put(",".join(str(w * a.cores_per_run + c) for c in range(a.cores_per_run)))

    def pinned(sv, s):
        cpus = slots.get()
        try:
            return run(sv, s, out, cpus=cpus)
        finally:
            slots.put(cpus)

    with ThreadPoolExecutor(a.workers) as ex:
        futs = {ex.submit(pinned, sv, s): (sv, s) for sv, s in jobs}
        for f in as_completed(futs):
            sv, s = futs[f]
            try:
                r = f.result()
                ok = r["returncode"] == 0 and "throughput" in r
            except Exception as e:  # keep the grid going
                r, ok = {"error": repr(e)}, False
            done += 1
            failed += not ok
            el = time.time() - t0
            print(f"[{done}/{len(jobs)} {el/60:.0f}m eta {el/done*(len(jobs)-done)/60:.0f}m] {sv} {s.key()} "
                  + (f"thr={r['throughput']:.3f} p99={r['lat_p99']:.0f}" if ok else f"FAILED {str(r.get('error'))[-200:]}"),
                  flush=True)
    print(f"done: {done} runs, {failed} failed", flush=True)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
