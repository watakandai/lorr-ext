"""Re-run contrasting configurations with full logs and render comparison videos.

    python -m lorrext.videos [--out results/videos] [--ticks 900]
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from .core import SOLVERS, Setting, run
from .replay import load, validate, video

PAIRS = [
    ("1_greedy_vs_oldest_first", "default2026", "fifo2026", Setting(arrival="poisson", rate=0.3, steps=1000)),
    ("2_winner_vs_runnerup", "nms2024", "kitty2024", Setting(arrival="poisson", rate=0.7, steps=1000)),
    ("3_lowload_winner_vs_runnerup", "nms2024", "kitty2024", Setting(arrival="poisson", rate=0.05, steps=1000)),
    ("4_lowload_2024_default_vs_winner", "default2024", "nms2024", Setting(arrival="poisson", rate=0.1, steps=1000)),
    ("5_kitchen_service_times", "default2026", "fifo2026", Setting(arrival="poisson", rate=0.2, service="kitchen", steps=1000)),
]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=Path("results/videos"))
    ap.add_argument("--runs", type=Path, default=Path("runs/replays"))
    ap.add_argument("--ticks", type=int, default=900)
    ap.add_argument("--only", default="")
    ap.add_argument("--cpus", default="12,13;14,15", help="';'-separated core sets, one per parallel run")
    a = ap.parse_args()
    pairs = [p for p in PAIRS if not a.only or p[0] in a.only.split(",")]
    uniq = {(sv, s.key()): (sv, s) for _, x, y, s in pairs for sv in (x, y)}
    jobs = sorted(uniq.values(), key=lambda j: SOLVERS[j[0]]["gen"])  # slow 2024 runs first
    import queue

    slots = queue.Queue()
    for c in a.cpus.split(";"):
        slots.put(c)

    def pinned(j):
        c = slots.get()
        try:
            return run(j[0], j[1], a.runs, screen=1, cpus=c)
        finally:
            slots.put(c)

    with ThreadPoolExecutor(slots.qsize()) as ex:
        rows = {(j[0], j[1].key()): r for j, r in zip(jobs, ex.map(pinned, jobs))}
    for name, x, y, s in pairs:
        ra, rb = rows[(x, s.key())], rows[(y, s.key())]
        A, B = load(Path(ra["output"]), label=SOLVERS[x]["label"]), load(Path(rb["output"]), label=SOLVERS[y]["label"])
        for lab, rp, r in ((x, A, ra), (y, B, rb)):
            print(f"{name}: {lab} p99={r['lat_p99']:.0f} thr={r['throughput']:.3f} replay check {validate(rp)}", flush=True)
        print(video(A, B, a.out / f"{name}.mp4", ticks=a.ticks, deadline=s.deadline), flush=True)


if __name__ == "__main__":
    main()
