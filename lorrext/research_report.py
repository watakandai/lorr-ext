"""Tables for the research grid (runs/research/rows): one per part, mean over seeds.

    python -m lorrext.research_report C2 B1 ...    # print markdown tables
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from .core import SOLVERS
from .research import OUT, grid

COLS = {"throughput": "tasks/tick", "lat_p50": "p50", "lat_p99": "p99", "miss_rate": "miss@300",
        "class0_miss": "urgent miss", "class1_miss": "standard miss", "breakdowns": "breakdowns",
        "frozen_waits": "frozen waits", "cascade_waits": "cascade waits", "all_stops": "all-stops"}


def rows(part: str) -> pd.DataFrame:
    keys = {(sv, s.key()): s for sv, s in grid()[part]}
    out = []
    for (sv, key), s in keys.items():
        f = OUT / "rows" / f"{sv}__{key}.json"
        if f.exists():
            r = json.loads(f.read_text())
            r["crashed"] = r["returncode"] != 0
            out.append(r)
    return pd.DataFrame(out)


def table(part: str, by: list[str], cols: list[str]) -> pd.DataFrame:
    df = rows(part)
    if df.empty:
        return df
    df["solver"] = df["solver"].map(lambda s: SOLVERS[s]["label"])
    for c in cols:
        if c not in df:
            df[c] = np.nan
        df.loc[df["crashed"], c] = np.nan
    agg = df.groupby(by + ["solver"]).agg({**{c: "mean" for c in cols}, "crashed": "sum", "seed": "count"})
    return agg.rename(columns={**COLS, "seed": "runs"}).round(3)


if __name__ == "__main__":
    for part in sys.argv[1:]:
        print(f"## {part}\n")
        print(table(part, ["layout", "agents", "tasks", "service", "arrival", "rate", "faults", "deadlines"],
                    ["throughput", "lat_p50", "lat_p99", "miss_rate"]).to_markdown())
