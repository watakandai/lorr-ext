"""Experiment grids for the research programme (docs/research/README.md).

    python -m lorrext.research --list                      # parts and run counts
    python -m lorrext.research --only C2,C3 --workers 6    # run some parts
    python -m lorrext.research                              # everything

Every run is cached in runs/research/rows/ (resumable). Each part answers one question:

    C2  map layouts: random, the same map renamed, warehouse (picking), sortation
    C3  fleet size on the random map: 100, 150, 200, 300, 400 robots
    B1  random breakdowns (MTBF 500..5000 ticks, repair 50-150), plus the 2024 kit's all-stop rule
    B2  permanently slow robots (10/30/50% of the fleet at half speed)
    B3  scheduled pauses (10% of each robot's time, in 20-tick blocks)
    C1  a busy stretch of aisle closed for 400 ticks
    A1  per-task deadline classes (20% urgent at D = 100, 80% standard at D = 400)
    A2  warehouse picking with slower stations
    A3  multi-stop tours (chain-store replenishment)
    A4  long pickup/drop-off trips (ride-hailing)
"""

from __future__ import annotations

import argparse
import queue
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

from .core import RESEARCH_VARIANTS, SOLVERS, Setting, run

OUT = Path("runs/research")
ALL = [s for s in SOLVERS if s not in RESEARCH_VARIANTS]
SEEDS = (0, 1)


def grid() -> dict[str, list[tuple[str, Setting]]]:
    g: dict[str, list[tuple[str, Setting]]] = {}
    layouts = [("random", "official", "none"), ("random_renamed", "official", "none"),
               ("warehouse", "g2p", "char:E:normal:8:0.3"), ("sortation", "sort", "char:E:uniform:2:4")]
    g["C2"] = [(sv, Setting(layout=L, tasks=t, service=svc, agents=n, seed=sd))
               for L, t, svc in layouts for n in (100, 200) for sd in SEEDS for sv in ALL]
    g["C3"] = [(sv, Setting(agents=n, seed=sd, **({"layout": "random"} if n in (100, 200, 400) else {})))
               for n in (100, 150, 200, 300, 400) for sd in SEEDS for sv in ALL]
    g["B1"] = [(sv, Setting(faults=f"bd{p}:50-150", seed=sd))
               for p in (0.0002, 0.0005, 0.001, 0.002) for sd in SEEDS for sv in ALL]
    g["B1"] += [(sv, Setting(faults=f"bd{p}:50-150+stopall", seed=sd))
                for p in (0.0002, 0.001) for sd in SEEDS for sv in ALL if SOLVERS[sv]["gen"] == "2024"]
    g["B2"] = [(sv, Setting(faults=f"slow{sh}x2", seed=sd)) for sh in (0.1, 0.3, 0.5) for sd in SEEDS for sv in ALL]
    g["B3"] = [(sv, Setting(faults="pause200x20", seed=sd)) for sd in SEEDS for sv in ALL]
    g["C1"] = [(sv, Setting(faults=f"close{c}@300-700", seed=sd))
               for c in ("hot8", "hot16") for sd in SEEDS for sv in ALL]
    g["A1"] = [(sv, Setting(arrival="poisson", rate=r, deadlines="0.2:100,0.8:400", seed=sd))
               for r in (0.2, 0.4, 0.6) for sd in SEEDS for sv in ALL]
    g["A2"] = [(sv, Setting(layout="warehouse", tasks="g2p", service=svc, arrival=arr, rate=r, seed=sd))
               for svc in ("none", "char:E:normal:8:0.3", "char:E:normal:20:0.3")
               for arr, r in (("pool", 0.5), ("poisson", 0.3), ("poisson", 0.6))
               for sd in SEEDS for sv in ALL]
    g["A3"] = [(sv, Setting(tasks=t, arrival=arr, rate=r, seed=sd, layout="random"))
               for t in ("uniform:2-5", "tour:5")
               for arr, r in (("pool", 0.5), ("poisson", 0.1), ("poisson", 0.2))
               for sd in SEEDS for sv in ALL]
    g["A4"] = [(sv, Setting(tasks="trip:25", arrival=arr, rate=r, seed=sd, layout="random"))
               for arr, r in (("pool", 0.5), ("poisson", 0.3), ("poisson", 0.6))
               for sd in SEEDS for sv in ALL]
    return g


def run_jobs(jobs, out: Path = OUT, workers: int = 6, cores_per_run: int = 2, log=print) -> list[dict]:
    """Run (solver, Setting) jobs on dedicated cores; slow 2024-kit runs first."""
    jobs = sorted(jobs, key=lambda j: SOLVERS[j[0]]["gen"] != "2024")
    slots: queue.Queue[str] = queue.Queue()
    for w in range(workers):
        slots.put(",".join(str(w * cores_per_run + c) for c in range(cores_per_run)))

    def pinned(sv, s):
        cpus = slots.get()
        try:
            return run(sv, s, out, cpus=cpus, timeout=2400)
        finally:
            slots.put(cpus)

    rows, t0 = [], time.time()
    with ThreadPoolExecutor(workers) as ex:
        futs = {ex.submit(pinned, sv, s): (sv, s) for sv, s in jobs}
        for i, f in enumerate(as_completed(futs), 1):
            sv, s = futs[f]
            r = f.result()
            rows.append(r)
            el = time.time() - t0
            log(f"[{i}/{len(jobs)} {el / 60:.0f}m eta {el / i * (len(jobs) - i) / 60:.0f}m] {sv:12s} {s.key():60s} "
                f"rc={r['returncode']} thr={r.get('throughput', float('nan')):.3f}")
    return rows


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", default="", help="comma list of parts")
    ap.add_argument("--solvers", default="", help="comma list of solvers")
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    g = grid()
    parts = a.only.split(",") if a.only else list(g)
    keep = set(a.solvers.split(",")) if a.solvers else set(ALL)
    jobs = [j for p in parts for j in g[p] if j[0] in keep]
    print(f"{len(jobs)} runs: " + ", ".join(f"{p}={sum(j[0] in keep for j in g[p])}" for p in parts), flush=True)
    if not a.list:
        run_jobs(jobs, workers=a.workers, log=lambda m: print(m, flush=True))


if __name__ == "__main__":
    main()
