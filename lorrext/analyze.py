"""Turn benchmark rows into the capacity / SLA-gap tables and plots.

    python -m lorrext.analyze --runs runs/bench --out results

Definitions (deadline D, default 300 ticks):
  * feasible(rate)  : mean over seeds of miss_D <= 1%, i.e. the 99th-percentile
                      latency of tasks that had the full D ticks is <= D
                      (tasks still open count as misses).
  * capacity_D      : largest tested arrival rate that is feasible (0 if none).
  * low-load failure: some tested rate *below* capacity_D is infeasible, i.e.
                      the solver meets the deadline at moderate load but not
                      when the system is nearly idle (reported separately).
  * pool throughput : tasks/tick in the competition's always-full pool mode.
  * SLA gap G       : 1 - capacity_D / pool throughput. The share of
                      leaderboard throughput you cannot use under the deadline.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import spearmanr

from .core import SOLVERS

MISS_OK = 0.01


def load(runs: Path) -> pd.DataFrame:
    rows = [json.loads(p.read_text()) for p in sorted((runs / "rows").glob("*.json"))]
    df = pd.DataFrame(rows)
    df = df[df.solver.isin(list(SOLVERS))]  # e.g. drop retired baselines from older runs
    df["ok"] = (df["returncode"] == 0) & df["throughput"].notna() if "throughput" in df else False
    df["scenario"] = df.apply(scenario, axis=1)
    return df


def scenario(r) -> str:
    if r["arrival"] == "pool":
        return "pool"
    s = r["arrival"]
    if r["service"] != "none":
        s += "+" + r["service"]
    if r["p_delay"] > 0:  # older runs used the kit's own uniform delays
        s += f"+delay{r['p_delay']:g}"
    d = r.get("delay", "none")
    if isinstance(d, str) and d != "none":
        s += f"+delay:{d}"
    return s


def capacity(df: pd.DataFrame, D: int) -> pd.DataFrame:
    """capacity per (solver, scenario) at deadline D."""
    out = []
    col = f"miss_{D}"
    for (sv, sc), g in df[(df.scenario != "pool") & df.ok].groupby(["solver", "scenario"]):
        m = g.groupby("rate")[col].mean().sort_index()
        feas = m[m <= MISS_OK]
        cap = float(feas.index.max()) if len(feas) else 0.0
        low_fail = [float(r) for r, miss in m.items() if r < cap and miss > MISS_OK]
        out.append({"solver": sv, "scenario": sc, "deadline": D, "capacity": cap,
                    "low_load_fail": ",".join(f"{r:g}" for r in low_fail),
                    "n_rates": len(m), "seeds": int(g.groupby("rate").size().min())})
    return pd.DataFrame(out, columns=["solver", "scenario", "deadline", "capacity", "low_load_fail", "n_rates", "seeds"])


def table(df: pd.DataFrame, D: int) -> pd.DataFrame:
    pool = df[(df.scenario == "pool") & df.ok].groupby("solver")["throughput"].agg(["mean", "std", "count"])
    c = capacity(df, D)
    cap = c.pivot(index="solver", columns="scenario", values="capacity")
    low = c[c.low_load_fail != ""]
    t = pool.rename(columns={"mean": "pool_thr", "std": "pool_thr_sd", "count": "pool_seeds"}).join(cap, how="outer")
    for sc in cap.columns:
        t[f"G[{sc}]"] = 1 - t[sc] / t["pool_thr"]
    t["low_load_fail"] = ["; ".join(f"{r.scenario}@{r.low_load_fail}" for r in low[low.solver == s].itertuples()) for s in t.index]
    t.insert(0, "label", [SOLVERS[s]["label"] for s in t.index])
    t.insert(1, "kit", [SOLVERS[s]["gen"] for s in t.index])
    return t.sort_values("pool_thr", ascending=False)


def rank_corr(t: pd.DataFrame, sc: str, subset=None) -> tuple[float, float, int]:
    x = t if subset is None else t.loc[[s for s in subset if s in t.index]]
    x = x[["pool_thr", sc]].dropna()
    if len(x) < 3 or x[sc].nunique() < 2 or x["pool_thr"].nunique() < 2:
        return float("nan"), float("nan"), len(x)
    rho, p = spearmanr(x["pool_thr"], x[sc])
    return float(rho), float(p), len(x)


def plot_latency(df: pd.DataFrame, out: Path, D: int = 300) -> None:
    scs = [s for s in ["poisson", "mmpp", "hawkes", "poisson+kitchen", "poisson+delay0.05", "mmpp+delay0.05",
                       "poisson+delay:kit_like", "poisson+delay:congestion"]
           if s in set(df.scenario)]
    fig, axes = plt.subplots(2, (len(scs) + 1) // 2, figsize=(5.2 * ((len(scs) + 1) // 2), 8), squeeze=False)
    cmap = plt.get_cmap("tab10")
    for ax, sc in zip(axes.flat, scs):
        g = df[(df.scenario == sc) & df.ok]
        for i, sv in enumerate(SOLVERS):
            m = g[g.solver == sv].groupby("rate")[f"miss_{D}"].mean()
            if len(m):
                ax.plot(m.index, np.maximum(m.values, 1e-3), "o-", ms=3, color=cmap(i),
                        ls="-" if SOLVERS[sv]["gen"] == "2026" else "--", label=SOLVERS[sv]["label"])
        ax.axhline(MISS_OK, color="k", lw=0.8, ls=":")
        ax.set_yscale("log")
        ax.set_ylim(8e-4, 1.2)
        ax.set_title(sc)
        ax.set_xlabel("arrival rate (tasks/tick)")
        ax.set_ylabel(f"share of tasks missing D={D} (0 shown at 1e-3)")
    for ax in axes.flat[len(scs):]:
        ax.axis("off")
    axes.flat[0].legend(fontsize=7)
    fig.suptitle(f"Deadline-miss rate vs load, D={D} (dotted: 1% = p99 at the deadline); solid = 2026 kit, dashed = 2024 kit")
    fig.tight_layout()
    fig.savefig(out / "miss_vs_rate.png", dpi=130)
    plt.close(fig)


def plot_rank(t: pd.DataFrame, out: Path, sc: str = "poisson", D: int = 300) -> None:
    fig, ax = plt.subplots(figsize=(6.5, 5))
    for sv, r in t.iterrows():
        ax.scatter(r["pool_thr"], r[sc], s=40, marker="o" if r["kit"] == "2026" else "s")
        ax.annotate(r["label"], (r["pool_thr"], r[sc]), fontsize=7, xytext=(4, 3), textcoords="offset points")
    lim = max(t["pool_thr"].max(), t[sc].max()) * 1.1
    ax.plot([0, lim], [0, lim], "k:", lw=0.8)
    ax.set_xlabel("competition throughput, pool mode (tasks/tick)")
    ax.set_ylabel(f"deadline-feasible capacity, {sc}, D={D} (tasks/tick)")
    ax.set_title("Leaderboard metric vs usable capacity (dotted: G = 0)")
    fig.tight_layout()
    fig.savefig(out / f"rank_{sc}.png", dpi=130)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", default="runs/bench")
    ap.add_argument("--out", default="results")
    ap.add_argument("--lowload", default="runs/lowload", help="long-horizon light-load runs (optional)")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    df = load(Path(a.runs))
    df.drop(columns=["output"], errors="ignore").to_csv(out / "runs.csv", index=False)
    lines = [f"# LoRR-Ext benchmark results\n", f"{len(df)} runs, {int((~df.ok).sum())} failed.\n"]
    if (~df.ok).any():
        lines.append("Failed: " + ", ".join(f"{r.solver}/{r.scenario}/r{r.rate}/s{r.seed}" for r in df[~df.ok].itertuples()) + "\n")
    for D in (200, 300, 400):
        t = table(df, D)
        t.to_csv(out / f"capacity_D{D}.csv")
        lines.append(f"\n## Deadline D = {D} ticks\n")
        lines.append(t.round(3).to_markdown() + "\n")
        for sc in [c for c in t.columns if c in set(df.scenario)]:
            rho, p, n = rank_corr(t, sc)
            rho24, _, n24 = rank_corr(t, sc, [s for s in SOLVERS if SOLVERS[s]["gen"] == "2024"])
            rho26, _, n26 = rank_corr(t, sc, [s for s in SOLVERS if SOLVERS[s]["gen"] == "2026"])
            lines.append(f"- Spearman(pool throughput, capacity[{sc}]): all {rho:+.2f} (p={p:.2f}, n={n}); "
                         f"2026 kit only {rho26:+.2f} (n={n26}); 2024 kit only {rho24:+.2f} (n={n24})\n")
        if D == 300:
            plot_rank(t, out, "poisson", D)
    plot_latency(df, out, 300)
    if (Path(a.lowload) / "rows").exists():
        lo = load(Path(a.lowload))
        lo.drop(columns=["output"], errors="ignore").to_csv(out / "lowload_runs.csv", index=False)
        tl = lo[lo.ok].groupby(["solver", "rate"])[["lat_p50", "miss_300", "miss_500", "open_at_end", "revealed"]].mean()
        tl = tl.unstack("rate").round(3)
        tl.insert(0, "label", [SOLVERS[s]["label"] for s in tl.index])
        steps = int(lo.steps.iloc[0])
        lines.append(f"\n## Light load, {steps}-tick horizon ({lo.seed.nunique()} seeds, Poisson)\n")
        lines.append("Tasks still open at the end are mostly robots that were assigned but never got through.\n\n")
        lines.append(tl.to_markdown() + "\n")
    (out / "summary.md").write_text("".join(lines))
    print("".join(lines))


if __name__ == "__main__":
    main()
