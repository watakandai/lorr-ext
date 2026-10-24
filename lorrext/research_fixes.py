"""Stage 5: run each fix from docs/research/proposals.md on the grid settings that exposed
the weakness it targets, so its rows can be compared one-to-one with the original solver's.

    python -m lorrext.research_fixes --workers 4

Rows go to runs/research/rows like the grid's (same cache keys, a different solver name),
except the short-window runs, which use another planner window and so go to
runs/research_w60 to keep them from being confused with 300 ms-window rows.
"""

from __future__ import annotations

import argparse

from .core import Setting
from .investigate import timing
from .research import OUT, grid, run_jobs

OUT_W60 = OUT.parent / "research_w60"


def settings(part: str, solver: str, keep=lambda s: True) -> list[Setting]:
    """The settings `solver` was run on in grid part `part` (both seeds), filtered by `keep`."""
    return [s for sv, s in grid()[part] if sv == solver and keep(s)]


def jobs() -> list[tuple[str, Setting]]:
    out = []
    # B1 root cause: the fault-aware 2024 default (planner told which robots are frozen) on
    # every robot-fault setting, plus the fault-free baseline to measure what is left.
    for part in ("B1", "B2", "B3", "C1"):
        out += [("default2024fa", s) for s in settings(part, "default2024")]
    out += [("default2024fa", s) for s in settings("C3", "default2024", lambda s: s.agents == 100)]
    # E2: the No Man's Sky arrival fix was measured only on the competition map. The livelock
    # needs free loops around the goal, which the open warehouse/sortation maps have, so check
    # the fix there, and that it does not hurt under breakdowns.
    out += [("nms2024fix", s) for s in settings("C2", "nms2024")]
    out += [("nms2024fix", s) for s in settings("B1", "nms2024", lambda s: "stopall" not in s.faults)]
    # E1: plan-ahead lets the 2026 rollout move a robot on to its next errand. Its gain was
    # small on 2-errand tasks; 5-stop tours (A3) have many more errand changes, so it should
    # matter more there. Large fleets (C3) are where the 2026 kit collapses.
    out += [("default2026nx", s) for s in settings("A3", "default2026", lambda s: s.arrival == "pool")]
    out += [("default2026nx", s) for s in settings("C3", "default2026", lambda s: s.agents >= 300)]
    return out


def jobs_w60() -> list[tuple[str, Setting]]:
    """E1's best fix: the 2026 default with a 2-tick (60 ms) window and a 20 ms PIBT reserve,
    on the tours, the large fleets and two breakdown rates. Run with core.WINDOW_MS = 60."""
    g = grid()
    return [("default2026g", s) for sv, s in g["A3"] + g["C3"] + g["B1"]
            if sv == "default2026" and (s.arrival == "pool" and s.tasks != "official" or s.agents >= 300
                                        or s.faults in ("bd0.0005:50-150", "bd0.002:50-150"))]


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    w = ap.parse_args().workers
    j = jobs()
    print(len(j), "runs", flush=True)
    run_jobs(j, workers=w, log=lambda m: print(m, flush=True))
    # the window length is a module global read by core.run, so these must run after the
    # others, not in parallel with them
    with timing(30, 60):
        run_jobs(jobs_w60(), out=OUT_W60, workers=w, log=lambda m: print(m, flush=True))
