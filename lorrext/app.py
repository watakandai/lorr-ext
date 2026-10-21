"""LoRR-Ext workbench: run sweeps, replay runs, diff two runs, plot results.

    uv run streamlit run lorrext/app.py
"""

from __future__ import annotations

import json
import subprocess
import sys
from dataclasses import asdict, fields
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import streamlit as st

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lorrext import analyze, arrivals  # noqa: E402
from lorrext.core import ROOT, SERVICE_PRESETS, SOLVERS, Setting, service_stops  # noqa: E402

RUNS = ROOT / "runs"
st.set_page_config(page_title="LoRR-Ext workbench", layout="wide")


# ------------------------------------------------------------------ data
@st.cache_data(ttl=20)
def load_rows() -> pd.DataFrame:
    rows = []
    for p in RUNS.glob("*/rows/*.json"):
        r = json.loads(p.read_text())
        r["sweep"] = p.parent.parent.name
        r["tag"] = p.stem
        rows.append(r)
    if not rows:
        return pd.DataFrame()
    df = pd.DataFrame(rows)
    df["ok"] = (df["returncode"] == 0) & df.get("throughput", pd.Series(dtype=float)).notna()
    df["scenario"] = df.apply(analyze.scenario, axis=1)
    df["label"] = df["solver"].map(lambda s: SOLVERS.get(s, {}).get("label", s))
    df["has_replay"] = df["output"].map(lambda o: isinstance(o, str) and Path(o).exists())
    return df


def task_records(sweep: str, tag: str) -> np.ndarray | None:
    p = RUNS / sweep / "tasks" / f"{tag}.npy"
    return np.load(p) if p.exists() else None


@st.cache_resource(max_entries=4)
def replay(path: str, label: str):
    from lorrext.replay import _index_events, load

    rp = load(Path(path), label=label)
    _index_events(rp)
    return rp


def draw_map(rp, t: int, deadline: int, ax, highlight_task: int | None = None):
    from lorrext.replay import robot_states

    ax.imshow(rp.obstacles, cmap="Greys", vmin=0, vmax=1.6, interpolation="nearest")
    for tid, tk in rp.tasks.items():
        if tk["reveal"] <= t and (tk["done"] < 0 or tk["done"] > t):
            k = sum(1 for e in tk["_ev"] if e[0] <= t)
            r, c = tk["locs"][min(k, len(tk["locs"]) - 1)]
            age = t - tk["reveal"]
            ax.scatter(c, r, marker="s", s=40, color=plt.cm.autumn_r(min(age / deadline, 1)),
                       edgecolors="k" if age > deadline else "none", linewidths=1.2)
    colors = np.array(["#9aa0a6", "#1c7ed6", "#f08c00"])
    ax.scatter(rp.pos[t][:, 1], rp.pos[t][:, 0], c=colors[robot_states(rp, t)], s=26, edgecolors="white", linewidths=0.4)
    if highlight_task is not None and highlight_task in rp.tasks:
        tk = rp.tasks[highlight_task]
        locs = np.array(tk["locs"])
        ax.plot(locs[:, 1], locs[:, 0], "m--", lw=1)
        ax.scatter(locs[:, 1], locs[:, 0], marker="*", s=120, color="m")
        a = rp.assign[t]
        for ag in np.where(a == highlight_task)[0]:
            ax.scatter(rp.pos[t][ag, 1], rp.pos[t][ag, 0], s=160, facecolors="none", edgecolors="m", linewidths=2)
    ax.set_xticks([]); ax.set_yticks([])


def task_history(rp, tid: int) -> pd.DataFrame:
    tk = rp.tasks[tid]
    ev = [("revealed", tk["reveal"], "")]
    holders = np.where((rp.assign == tid).any(axis=0))[0]
    for ag in holders:
        ts = np.where(rp.assign[:, ag] == tid)[0]
        ev.append((f"assigned to robot {ag}", int(ts[0]), f"held until {int(ts[-1])}"))
    for t, a, _, seq in sorted(tk["_ev"]):
        ev.append((f"errand {seq}/{len(tk['locs'])} done", t, f"robot {a} at {tk['locs'][seq-1]}"))
    if tk["done"] < 0:
        ev.append(("still open at end", rp.T, ""))
    return pd.DataFrame(ev, columns=["event", "tick", "detail"]).sort_values("tick")


# ------------------------------------------------------------------ screens
df = load_rows()
tab_runs, tab_new, tab_replay, tab_cmp, tab_res = st.tabs(["Runs", "New sweep", "Replay", "Compare & diff", "Results"])

with tab_runs:
    st.subheader("Runs")
    if df.empty:
        st.info("No runs yet. Start one under **New sweep** or run `python -m lorrext.bench`.")
    else:
        c1, c2, c3, c4 = st.columns(4)
        sweeps = c1.multiselect("sweep", sorted(df.sweep.unique()), default=sorted(df.sweep.unique()))
        solv = c2.multiselect("solver", sorted(df.solver.unique()), default=sorted(df.solver.unique()))
        scen = c3.multiselect("scenario", sorted(df.scenario.unique()), default=sorted(df.scenario.unique()))
        only_replay = c4.checkbox("only runs with replay logs")
        v = df[df.sweep.isin(sweeps) & df.solver.isin(solv) & df.scenario.isin(scen)]
        if only_replay:
            v = v[v.has_replay]
        cols = ["sweep", "label", "scenario", "rate", "seed", "throughput", "lat_p50", "lat_p99", "miss_300",
                "open_at_end", "wall_s", "has_replay", "ok", "tag"]
        v = v[[c for c in cols if c in v]].sort_values(["sweep", "scenario", "rate", "label"])
        sel = st.dataframe(v, hide_index=True, width="stretch", on_select="rerun", selection_mode="multi-row")
        picked = v.iloc[sel.selection.rows] if sel.selection.rows else v.iloc[:0]
        st.caption(f"{len(v)} runs shown; {len(picked)} selected. Pick two and open **Compare & diff**.")
        if len(picked) >= 1:
            st.session_state["picked"] = list(zip(picked.sweep, picked.tag))

with tab_new:
    st.subheader("New sweep")
    left, right = st.columns([1, 1])
    with left:
        name = st.text_input("sweep name", "my_sweep")
        arrival = st.selectbox("arrival model", ["pool", "poisson", "mmpp", "hawkes"],
                               help="pool = the competition's always-full task pool (numTasksReveal x robots open at all times)")
        s0 = Setting(arrival=arrival)
        if arrival == "mmpp":
            s0.burst_factor = st.slider("burst rate / mean rate", 1.0, 10.0, 4.0, 0.5)
            s0.burst_fraction = st.slider("share of time in burst", 0.05, 0.5, 0.2, 0.05)
            s0.burst_length = st.slider("mean burst length (ticks)", 5.0, 200.0, 50.0, 5.0)
        if arrival == "hawkes":
            s0.branching = st.slider("branching ratio (offspring per task)", 0.0, 0.95, 0.6, 0.05)
            s0.decay = st.slider("excitation decay per tick", 0.005, 0.5, 0.05, 0.005)
        rates = st.multiselect("arrival rates (tasks/tick)", [0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0, 1.2],
                               default=[0.1, 0.3, 0.5]) if arrival != "pool" else [0.0]
        preset = st.selectbox("service time per errand", list(SERVICE_PRESETS) + ["custom"],
                              help="how long a robot must stay on an errand cell; errand j of a task uses slot j mod n")
        service = st.text_input("custom spec: `lo-hi` or `dist:mean[:cv[:min[:max]]]`, slots separated by `;` "
                                "(dist = uniform, fixed, exponential, lognormal, gamma, normal)",
                                "lognormal:3:0.3;gamma:20:0.5") if preset == "custom" else preset
        try:
            stops = service_stops(service)
        except Exception as e:  # noqa: BLE001
            st.error(f"bad service spec: {e}")
            stops = []
        p_delay = st.slider("move-delay probability (2026 kit only)", 0.0, 0.2, 0.0, 0.01)
        seeds = st.number_input("seeds", 1, 10, 1)
        steps = st.number_input("ticks per run", 200, 5000, 1000, 100)
        deadline = st.number_input("deadline (ticks)", 50, 2000, 300, 50)
        replay_logs = st.checkbox("keep full logs (needed for Replay / video)", value=False)
    with right:
        st.markdown("**Methods**")
        chosen = [k for k, v_ in SOLVERS.items()
                  if st.checkbox(f"{v_['label']}  ·  {v_['gen']} kit", value=k in ("default2026", "fifo2026", "nms2024"), key=f"m_{k}")]
        if p_delay > 0 and any(SOLVERS[c]["gen"] == "2024" for c in chosen):
            st.warning("The 2024 kit has no move-delay model; 2024 entries will run without delays.")
        if any(SOLVERS[c]["gen"] == "2024" for c in chosen) and any(SOLVERS[c]["gen"] == "2026" for c in chosen):
            st.warning("Mixing kits: the 2026 kit runs in real time (30 ms ticks, 300 ms planning window) and the 2024 kit "
                       "gives 100 ms per tick. Its own default loses about half its throughput to the kit, so compare within a kit.")
        n = len(chosen) * len(rates) * seeds
        est = sum((steps * 0.035 + 8) if SOLVERS[c]["gen"] == "2026" else (steps * 0.07 + 10) for c in chosen) * len(rates) * seeds
        st.metric("runs", n)
        st.metric("estimated time with 5 workers", f"{est / 5 / 60:.0f} min")
        if stops:
            fig, ax = plt.subplots(figsize=(6, 2.2))
            for j, stop in enumerate(stops):
                x = arrivals.service_samples(stop)
                ax.hist(x, bins=np.arange(x.min(), x.max() + 2) - 0.5, alpha=0.6, density=True,
                        label=f"slot {j}: {stop['dist']} (mean {x.mean():.1f}, p95 {np.percentile(x, 95):.0f})")
            ax.set_xlabel("service time (ticks)"); ax.set_ylabel("probability"); ax.legend(fontsize=7)
            ax.set_title("service-time distributions (what the robot must wait at each errand)", fontsize=9)
            st.pyplot(fig, width="stretch")
        if arrival != "pool":
            s_prev = Setting(**{**asdict(s0), "rate": max(rates) if rates else 0.3, "steps": steps})
            x = arrivals.sample(s_prev, steps)
            fig, ax = plt.subplots(figsize=(6, 2.2))
            ax.bar(np.arange(len(x)), x, width=1.0, color="#1c7ed6")
            ax.set_xlabel("tick"); ax.set_ylabel("new tasks")
            ax.set_title(f"sampled arrivals at rate {s_prev.rate:g} (preview)", fontsize=9)
            st.pyplot(fig, width="stretch")
            st.json(arrivals.burst_stats(x))
        if st.button("Launch sweep", type="primary", disabled=n == 0):
            spec = {"name": name, "solvers": chosen, "rates": rates, "seeds": int(seeds),
                    "setting": {**asdict(s0), "service": service, "p_delay": p_delay, "steps": int(steps),
                                "deadline": int(deadline)}, "screen": 1 if replay_logs else 3}
            out = RUNS / name
            out.mkdir(parents=True, exist_ok=True)
            (out / "sweep.json").write_text(json.dumps(spec, indent=1))
            log = open(out / "sweep.log", "a")
            subprocess.Popen([sys.executable, "-m", "lorrext.sweep", str(out / "sweep.json")], cwd=ROOT,
                             stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            st.success(f"Launched {n} runs into runs/{name}/ (log: runs/{name}/sweep.log). Rows appear under Runs as they finish.")

with tab_replay:
    st.subheader("Replay")
    rr = df[df.has_replay] if not df.empty else df
    if rr.empty:
        st.info("No runs with full logs. Launch a sweep with **keep full logs**, or `python -m lorrext.videos`.")
    else:
        opts = {f"{r.sweep} · {r.label} · {r.scenario} r{r.rate:g} s{r.seed}": r for r in rr.itertuples()}
        key = st.selectbox("run", list(opts))
        r = opts[key]
        rp = replay(r.output, r.label)
        D = int(r.deadline)
        lat = {tid: (tk["done"] if tk["done"] >= 0 else rp.T) - tk["reveal"] for tid, tk in rp.tasks.items()}
        worst = max(lat, key=lat.get)
        c1, c2 = st.columns([3, 2])
        with c2:
            if st.button("jump to worst task"):
                st.session_state["rp_task"] = worst
                st.session_state["rp_t"] = min(rp.tasks[worst]["reveal"] + D, rp.T)
            tid = st.number_input("task id", 0, max(rp.tasks), int(st.session_state.get("rp_task", worst)))
            st.caption(f"latency {lat.get(tid, '?')} ticks (deadline {D})")
            if tid in rp.tasks:
                st.dataframe(task_history(rp, tid), hide_index=True, width="stretch")
        with c1:
            t = st.slider("tick", 0, rp.T, int(st.session_state.get("rp_t", min(300, rp.T))))
            fig, ax = plt.subplots(figsize=(6, 6))
            draw_map(rp, t, D, ax, highlight_task=tid)
            st.pyplot(fig, width="stretch")
            misses = sorted(tk["reveal"] + D for tk in rp.tasks.values()
                            if (tk["done"] < 0 and tk["reveal"] + D <= rp.T) or tk["done"] - tk["reveal"] > D)
            fig2, ax2 = plt.subplots(figsize=(6, 0.9))
            ax2.eventplot(misses, colors="r", lineoffsets=0, linelengths=1)
            ax2.axvline(t, color="k", lw=0.8); ax2.set_xlim(0, rp.T); ax2.set_yticks([])
            ax2.set_title(f"deadline misses over time ({len(misses)})", fontsize=8)
            st.pyplot(fig2, width="stretch")

with tab_cmp:
    st.subheader("Compare & diff")
    picked = st.session_state.get("picked", [])
    if df.empty or len(picked) < 2:
        st.info("Select two runs in **Runs** (same seed and arrival settings gives a task-by-task diff).")
    else:
        (sa, ta), (sb, tb) = picked[:2]
        A = df[(df.sweep == sa) & (df.tag == ta)].iloc[0]
        B = df[(df.sweep == sb) & (df.tag == tb)].iloc[0]
        st.markdown(f"**A** = {A.label} · {A.scenario} r{A.rate:g} s{A.seed}   vs   **B** = {B.label} · {B.scenario} r{B.rate:g} s{B.seed}")
        mcols = st.columns(5)
        for c, (m, better) in zip(mcols, [("throughput", 1), ("lat_p50", -1), ("lat_p99", -1), ("miss_300", -1), ("open_at_end", -1)]):
            c.metric(m, f"{B[m]:.3g}", f"{B[m] - A[m]:+.3g} vs A", delta_color="normal" if better > 0 else "inverse")
        diff = {f.name: (A[f.name], B[f.name]) for f in fields(Setting) if A[f.name] != B[f.name]}
        diff["solver"] = (A.solver, B.solver)
        st.markdown("**Settings that differ**")
        st.table(pd.DataFrame(diff, index=["A", "B"]).T)
        ra, rb = task_records(sa, ta), task_records(sb, tb)
        if ra is not None and rb is not None:
            T = int(A.steps)
            da = pd.DataFrame(ra, columns=["task", "reveal", "done", "agent", "stops"]).set_index("task")
            db = pd.DataFrame(rb, columns=["task", "reveal", "done", "agent", "stops"]).set_index("task")
            j = da.join(db, lsuffix="_A", rsuffix="_B", how="inner")
            same_arrivals = (j.reveal_A == j.reveal_B).mean()
            j["lat_A"] = np.where(j.done_A >= 0, j.done_A - j.reveal_A, np.nan)
            j["lat_B"] = np.where(j.done_B >= 0, j.done_B - j.reveal_B, np.nan)
            j["B_minus_A"] = j.lat_B - j.lat_A
            st.caption(f"{len(j)} common task ids; reveal times identical for {same_arrivals:.0%}"
                       + ("" if same_arrivals > 0.99 else " (different arrivals: per-task diff is not like-for-like)"))
            fig, ax = plt.subplots(figsize=(7, 2.4))
            ax.plot(j.reveal_A, j.B_minus_A, ".", ms=3)
            ax.axhline(0, color="k", lw=0.6)
            ax.set_xlabel("reveal tick"); ax.set_ylabel("latency B − A")
            ax.set_title("where the runs diverge", fontsize=9)
            st.pyplot(fig, width="stretch")
            st.markdown("**Tasks sorted by how much later B finished**")
            st.dataframe(j.sort_values("B_minus_A", ascending=False, na_position="first").head(200), width="stretch")
        if A.has_replay and B.has_replay:
            rpa, rpb = replay(A.output, f"A: {A.label}"), replay(B.output, f"B: {B.label}")
            t = st.slider("tick (lockstep)", 0, min(rpa.T, rpb.T), min(300, rpa.T))
            c1, c2 = st.columns(2)
            for c, rp in ((c1, rpa), (c2, rpb)):
                fig, ax = plt.subplots(figsize=(5, 5))
                draw_map(rp, t, int(A.deadline), ax)
                ax.set_title(rp.label, fontsize=9)
                c.pyplot(fig, width="stretch")
            if st.button("export side-by-side MP4"):
                from lorrext.replay import video

                out = RUNS / "videos" / f"{ta}__vs__{tb}.mp4"
                with st.spinner("rendering..."):
                    video(rpa, rpb, out, ticks=min(900, rpa.T), deadline=int(A.deadline))
                st.video(str(out))
        else:
            st.caption("Map playback needs full logs for both runs.")

with tab_res:
    st.subheader("Results")
    if df.empty:
        st.info("No results yet.")
    else:
        sweep = st.selectbox("sweep", sorted(df.sweep.unique()), index=sorted(df.sweep.unique()).index("bench") if "bench" in set(df.sweep) else 0)
        d = df[(df.sweep == sweep) & df.ok]
        D = st.select_slider("deadline (ticks)", [100, 150, 200, 250, 300, 400, 500], 300)
        c1, c2, c3 = st.columns(3)
        scen = c1.selectbox("scenario", sorted(set(d.scenario) - {"pool"}) or ["pool"])
        y = c2.selectbox("y", [f"p99_{D}", f"miss_{D}", "lat_p50", "lat_mean", "throughput", "open_at_end"])
        group = c3.selectbox("colour by", ["label", "seed"])
        g = d[d.scenario == scen]
        fig, ax = plt.subplots(figsize=(8, 4))
        for name_, gg in g.groupby(group):
            m = gg.groupby("rate")[y].mean()
            ax.plot(m.index, np.minimum(m.values, 2000), "o-", ms=3, label=str(name_))
        if y.startswith("p99"):
            ax.axhline(D, color="k", ls=":", lw=0.9)
        ax.set_xlabel("arrival rate (tasks/tick)"); ax.set_ylabel(y); ax.legend(fontsize=7)
        st.pyplot(fig, width="stretch")
        if "pool" in set(d.scenario):
            t = analyze.table(d, D)
            st.markdown(f"**Capacity and SLA gap at D = {D}** (capacity: highest tested rate with ≤1% deadline misses at "
                        "it and every lower rate; G = 1 − capacity / pool throughput)")
            st.dataframe(t.round(3), width="stretch")
            if scen in t:
                rho, p, n = analyze.rank_corr(t, scen)
                st.caption(f"Spearman(pool throughput, capacity[{scen}]) = {rho:+.2f} (n={n}, p={p:.2f})")
                fig, ax = plt.subplots(figsize=(6, 4))
                order_lb = t["pool_thr"].rank(ascending=False)
                order_cap = t[scen].rank(ascending=False, method="min")
                for s_ in t.index:
                    ax.plot([0, 1], [order_lb[s_], order_cap[s_]], "o-")
                    ax.text(-0.03, order_lb[s_], t.loc[s_, "label"], ha="right", va="center", fontsize=7)
                    ax.text(1.03, order_cap[s_], t.loc[s_, "label"], ha="left", va="center", fontsize=7)
                ax.set_xticks([0, 1], ["leaderboard rank", f"deadline-capacity rank ({scen})"])
                ax.invert_yaxis(); ax.set_yticks([]); ax.set_xlim(-0.8, 1.8)
                st.pyplot(fig, width="stretch")
