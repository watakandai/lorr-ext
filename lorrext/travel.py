"""How long does driving actually take, compared with the map distance?

For every *leg* a robot drives (from getting a task, or from finishing an
errand, to arriving at the next errand) we compare three numbers:

    d        grid distance (moves only), what the schedulers use as a heuristic
    d_free   exact time on an empty map, including turns (1 tick per turn)
    actual   ticks it really took in the run

actual - d_free is the time lost to other robots (waiting, detours, being
pushed). Quantiles of `actual` per distance give a prediction interval.

    python -m lorrext.travel OUT.json [OUT2.json ...]
"""

from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path

import numpy as np
import pandas as pd

from .core import bfs_from
from .replay import _index_events, load

DR, DC = (0, 1, 0, -1), (1, 0, -1, 0)
_ROT: dict = {}


def free_time_to(target: int, free: np.ndarray) -> np.ndarray:
    """(rows*cols, 4) array: fewest ticks from (cell, heading) to reach `target`
    on an empty map (F = 1 tick, a 90° turn = 1 tick)."""
    if target in _ROT:
        return _ROT[target]
    h, w = free.shape
    dist = np.full((h * w, 4), np.iinfo(np.int32).max, dtype=np.int64)
    q = deque()
    for o in range(4):
        dist[target, o] = 0
        q.append((target, o))
    while q:  # reverse BFS: who can reach (v, o) in one action?
        v, o = q.popleft()
        r, c = divmod(v, w)
        preds = [(v, (o + 3) % 4), (v, (o + 1) % 4)]  # turned R / C into o
        pr, pc = r - DR[o], c - DC[o]                  # moved F along o into v
        if 0 <= pr < h and 0 <= pc < w and free[pr, pc]:
            preds.append((pr * w + pc, o))
        for u, uo in preds:
            if dist[u, uo] > dist[v, o] + 1:
                dist[u, uo] = dist[v, o] + 1
                q.append((u, uo))
    _ROT[target] = dist
    return dist


def legs(path: Path) -> pd.DataFrame:
    rp = load(path)
    _index_events(rp)
    free = ~rp.obstacles
    w = rp.cols
    rows = []
    for tid, tk in rp.tasks.items():
        evs = sorted(tk["_ev"])
        for k, (t_ev, a, _, seq) in enumerate(evs):
            goal = tuple(tk["locs"][seq - 1])
            if k == 0:  # first leg starts when this robot took the task
                held = np.where(rp.assign[:, a] == tid)[0]
                if not len(held):
                    continue
                start = int(held[0])
            else:
                start = evs[k - 1][0]
            on = [u for u in range(start, t_ev + 1) if tuple(rp.pos[u, a]) == goal]
            if not on:
                continue
            arrive = on[0]
            v0 = int(rp.pos[start, a, 0]) * w + int(rp.pos[start, a, 1])
            g = goal[0] * w + goal[1]
            d = bfs_from(g).get(v0)
            if d is None:
                continue
            busy = int((rp.assign[start] >= 0).sum())
            rows.append({"task": tid, "errand": seq, "first_leg": k == 0, "start": start, "d": d,
                         "d_free": int(free_time_to(g, free)[v0, rp.ori[start, a]]),
                         "actual": arrive - start, "busy_robots": busy})
    df = pd.DataFrame(rows)
    if len(df):
        df["lost_to_others"] = df.actual - df.d_free
    return df


def interval_table(df: pd.DataFrame, bins=(0, 5, 10, 15, 20, 30, 45, 70)) -> pd.DataFrame:
    """Quantiles of actual travel time per grid-distance bin: a prediction interval."""
    cut = pd.cut(df.d, bins=list(bins), right=False)
    g = df.groupby(cut, observed=True).actual
    return pd.DataFrame({"legs": g.size(), "q05": g.quantile(0.05), "q50": g.median(), "q95": g.quantile(0.95),
                         "mean_d": df.groupby(cut, observed=True).d.mean()}).round(1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("outs", nargs="+", type=Path)
    a = ap.parse_args()
    for p in a.outs:
        df = legs(p)
        print(p.stem, f"{len(df)} legs; actual/d median {np.median(df.actual / df.d.clip(lower=1)):.2f}; "
              f"time lost to other robots: median {df.lost_to_others.median():.0f}, p95 {df.lost_to_others.quantile(0.95):.0f} ticks")
        print(interval_table(df).to_string())


if __name__ == "__main__":
    main()
