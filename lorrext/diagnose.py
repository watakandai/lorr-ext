"""Where does a late task's time go? Decomposes each task's latency from a full
log into: waiting unassigned, assigned robot moving, assigned robot stuck (not
on its errand and not moving), and dwelling on an errand. For stuck ticks it
also records whether an idle robot was adjacent (the "idle robots block busy
ones" hypothesis) versus being stuck among busy robots only.

    python -m lorrext.diagnose OUT.json [OUT2.json ...] [--deadline 300]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from .replay import _index_events, load


def decompose(path: Path, deadline: int = 300, late_only: bool = True) -> pd.DataFrame:
    rp = load(path)
    _index_events(rp)
    T = rp.T
    idle = rp.assign < 0                     # (T+1, A)
    moved = np.zeros_like(idle)
    moved[1:] = (rp.pos[1:] != rp.pos[:-1]).any(axis=2)
    occ = {}                                 # (t, r, c) -> agent, built lazily per tick
    rows = []
    for tid, tk in rp.tasks.items():
        end = tk["done"] if tk["done"] >= 0 else T
        lat = end - tk["reveal"]
        if late_only and lat <= deadline:
            continue
        if tk["reveal"] < 0.1 * T:
            continue
        c = dict(task=tid, latency=lat, open=tk["done"] < 0, unassigned=0, moving=0, dwelling=0,
                 stuck=0, stuck_next_to_idle=0, reassigned=0)
        holders = set()
        evs = sorted(tk["_ev"])
        for t in range(tk["reveal"] + 1, end + 1):
            ags = np.where(rp.assign[t] == tid)[0]
            if len(ags) == 0:
                c["unassigned"] += 1
                continue
            a = int(ags[0])
            holders.add(a)
            k = sum(1 for e in evs if e[0] < t)
            goal = tk["locs"][min(k, len(tk["locs"]) - 1)]
            if tuple(rp.pos[t, a]) == tuple(goal):
                c["dwelling"] += 1
            elif moved[t, a]:
                c["moving"] += 1
            else:
                c["stuck"] += 1
                if t not in occ:
                    occ[t] = {tuple(p): i for i, p in enumerate(rp.pos[t])}
                r, cc = rp.pos[t, a]
                if any(occ[t].get((r + dr, cc + dc), -1) >= 0 and idle[t, occ[t][(r + dr, cc + dc)]]
                       for dr, dc in ((0, 1), (1, 0), (0, -1), (-1, 0))):
                    c["stuck_next_to_idle"] += 1
        c["reassigned"] = max(0, len(holders) - 1)
        rows.append(c)
    return pd.DataFrame(rows)


def summary(df: pd.DataFrame) -> dict:
    if df.empty:
        return {"late_tasks": 0}
    tot = df[["unassigned", "moving", "dwelling", "stuck"]].sum()
    share = (tot / tot.sum()).round(3).to_dict()
    return {"late_tasks": len(df), "open_at_end": int(df.open.sum()), **{f"share_{k}": v for k, v in share.items()},
            "stuck_next_to_idle_of_stuck": round(df.stuck_next_to_idle.sum() / max(df.stuck.sum(), 1), 3),
            "late_tasks_ever_unassigned_>50": int((df.unassigned > 50).sum()),
            "mean_reassignments": round(df.reassigned.mean(), 2)}


def idle_on_errands(path: Path) -> float:
    """Average share of idle robots parked on a cell that is some open task's next errand."""
    rp = load(path)
    _index_events(rp)
    shares = []
    for t in range(int(0.1 * rp.T), rp.T, 10):
        cells = set()
        for tk in rp.tasks.values():
            if tk["reveal"] <= t and (tk["done"] < 0 or tk["done"] > t):
                k = sum(1 for e in tk["_ev"] if e[0] <= t)
                cells.add(tuple(tk["locs"][min(k, len(tk["locs"]) - 1)]))
        idle = np.where(rp.assign[t] < 0)[0]
        if len(idle):
            shares.append(np.mean([tuple(rp.pos[t, a]) in cells for a in idle]))
    return float(np.mean(shares)) if shares else float("nan")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("outs", nargs="+", type=Path)
    ap.add_argument("--deadline", type=int, default=300)
    a = ap.parse_args()
    for p in a.outs:
        print(p.stem, summary(decompose(p, a.deadline)), "idle_on_open_errand_cells", round(idle_on_errands(p), 3))


if __name__ == "__main__":
    main()
