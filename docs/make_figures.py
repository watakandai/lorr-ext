"""Generate the figures used in docs/explainer.md.

    uv run python docs/make_figures.py

Needs the full-log pool runs in runs/explain/ (see the explainer) and, for the
capacity figure, the benchmark rows in results/runs.csv.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mp
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from lorrext import arrivals  # noqa: E402
from lorrext.core import Setting, bfs_from, service_stops, task_length  # noqa: E402
from lorrext.replay import _index_events, load, read_map  # noqa: E402

OUT = ROOT / "docs" / "figures"
OUT.mkdir(parents=True, exist_ok=True)
EXPLAIN = ROOT / "runs" / "explain" / "out"
MAP = read_map(ROOT / "instances" / "base" / "maps" / "random-32-32-20.map")
BLUE, ORANGE, GREY, RED, GREEN, PURPLE = "#1c7ed6", "#f08c00", "#9aa0a6", "#e03131", "#2f9e44", "#ae3ec9"
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})


def save(fig, name):
    fig.savefig(OUT / name, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print("wrote", OUT / name)


def box(ax, xy, w, h, text, color, fs=10, alpha=0.18):
    ax.add_patch(mp.FancyBboxPatch(xy, w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                   fc=color, ec=color, alpha=alpha, lw=1.5))
    ax.add_patch(mp.FancyBboxPatch(xy, w, h, boxstyle="round,pad=0.02,rounding_size=0.08", fc="none", ec=color, lw=1.5))
    ax.text(xy[0] + w / 2, xy[1] + h / 2, text, ha="center", va="center", fontsize=fs)


def arrow(ax, a, b, text="", color="k", rad=0.0, fs=8.5, off=(0, 0)):
    ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="-|>", color=color, lw=1.4,
                                                     connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((a[0] + b[0]) / 2 + off[0], (a[1] + b[1]) / 2 + off[1], text, fontsize=fs, ha="center", va="center",
                color=color, bbox=dict(fc="white", ec="none", pad=1))


# --------------------------------------------------------------------- 1
def fig_system_loop():
    fig, ax = plt.subplots(figsize=(11, 5.2))
    ax.set_xlim(0, 11); ax.set_ylim(0, 5.4); ax.axis("off")
    box(ax, (0.2, 3.3), 2.6, 1.5, "Task manager\n(open tasks = the pool)\n\nsrc/TaskManager.cpp", GREEN)
    box(ax, (4.1, 3.3), 2.8, 1.5, "Scheduler\nwhich robot does\nwhich task?\nsrc/TaskScheduler.cpp", BLUE)
    box(ax, (8.2, 3.3), 2.6, 1.5, "Planner\nwhich move does each\nrobot make next?\nsrc/MAPFPlanner.cpp", PURPLE)
    box(ax, (4.1, 0.4), 2.8, 1.5, "Simulator / executor\napplies moves, checks\ncollisions, advances time\nsrc/Simulator.cpp", ORANGE)
    arrow(ax, (2.8, 4.05), (4.1, 4.05), "open tasks,\nfree robots", off=(0, 0.35))
    arrow(ax, (6.9, 4.05), (8.2, 4.05), "goal of each\nrobot", off=(0, 0.35))
    arrow(ax, (9.5, 3.3), (6.9, 1.4), "planned actions\n(F, R, C, W)", rad=-0.2, off=(0.5, -0.1))
    arrow(ax, (4.1, 1.15), (1.5, 3.3), "robot positions →\nerrand reached?\ntask finished?", rad=-0.25, off=(-0.6, -0.2))
    # LoRR-Ext additions
    kw = dict(fontsize=8.5, color=RED, ha="left", va="top")
    ax.text(0.2, 3.15, "LoRR-Ext ①  arrivals: when do new tasks appear?\n(pool = keep 150 open; or Poisson / bursty)", **kw)
    ax.text(0.2, 1.75, "LoRR-Ext ②  service time:\nrobot must stay d ticks on\nthe errand cell (d ~ known\ndistribution, value hidden)", **kw)
    ax.text(7.25, 1.35, "LoRR-Ext ③  per-task records:\nrevealed, completed → latency,\ndeadline misses, CVaR …", **kw)
    ax.text(5.5, 5.25, "One tick of the simulation loop (repeats every tick; the planner may plan several ticks ahead)",
            ha="center", fontsize=10.5, weight="bold")
    save(fig, "01_system_loop.png")


# --------------------------------------------------------------------- 2
def fig_world():
    rp = load(EXPLAIN / "nms2024__pool1.5_none_d0_s0_T1000.json")
    _index_events(rp)
    t = 200
    fig, ax = plt.subplots(figsize=(7.2, 7.2))
    ax.imshow(MAP, cmap="Greys", vmin=0, vmax=1.6, interpolation="nearest")
    ax.scatter(rp.pos[t][:, 1], rp.pos[t][:, 0], s=22, color=BLUE, zorder=3, label="robot (100)")
    # one task with 4 errands drawn with its shortest path
    tid = next(i for i, tk in rp.tasks.items() if len(tk["locs"]) == 4 and tk["reveal"] > 50)
    locs = rp.tasks[tid]["locs"]
    w = MAP.shape[1]
    for (r1, c1), (r2, c2) in zip(locs, locs[1:]):
        path = shortest_path(r1 * w + c1, r2 * w + c2, w)
        ax.plot([p % w for p in path], [p // w for p in path], color=RED, lw=2, alpha=0.7)
    for k, (r, c) in enumerate(locs, 1):
        ax.scatter(c, r, marker="s", s=160, color=RED, zorder=4)
        ax.text(c, r, str(k), color="white", ha="center", va="center", fontsize=8, weight="bold", zorder=5)
    ax.set_title(f"The world: 32×32 grid, black = obstacle, 100 robots.\nRed: one task = an ordered list of {len(locs)} errands "
                 f"(cells to visit);\nline = shortest route between them ({task_length(tid)} moves).", fontsize=10)
    ax.set_xticks([]); ax.set_yticks([])
    ax.legend(loc="lower left", fontsize=8)
    save(fig, "02_world.png")


def shortest_path(a, b, w):
    d = bfs_from(b)
    path, cur = [a], a
    while cur != b:
        r, c = divmod(cur, w)
        cur = min((n for n in (cur + 1, cur - 1, cur + w, cur - w) if n in d and abs(n % w - c) <= 1), key=d.get)
        path.append(cur)
    return path


# --------------------------------------------------------------------- 3
def fig_actions():
    fig, axes = plt.subplots(1, 4, figsize=(13, 3.4))
    def grid(ax, n=3):
        ax.set_xlim(-0.5, n - 0.5); ax.set_ylim(n - 0.5, -0.5); ax.set_aspect("equal")
        for i in range(n + 1):
            ax.axhline(i - 0.5, color="#ddd", lw=1); ax.axvline(i - 0.5, color="#ddd", lw=1)
        ax.set_xticks([]); ax.set_yticks([])
    def robot(ax, x, y, ori, color=BLUE, label=None):
        ax.add_patch(mp.Circle((x, y), 0.3, color=color, zorder=3))
        dx, dy = [(0.3, 0), (0, 0.3), (-0.3, 0), (0, -0.3)][ori]
        ax.annotate("", xy=(x + dx * 1.5, y + dy * 1.5), xytext=(x, y), arrowprops=dict(arrowstyle="-|>", color="k", lw=1.5), zorder=4)
        if label:
            ax.text(x, y, label, color="white", ha="center", va="center", fontsize=8, weight="bold", zorder=5)
    ax = axes[0]; grid(ax); robot(ax, 0, 1, 0)
    ax.annotate("", xy=(1, 1), xytext=(0.35, 1), arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=2, ls="--"))
    ax.set_title("F = forward one cell\n(in the direction it faces)", fontsize=9.5)
    ax = axes[1]; grid(ax); robot(ax, 1, 1, 0)
    ax.annotate("", xy=(1.1, 1.55), xytext=(1.55, 1.05), arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=2, connectionstyle="arc3,rad=-0.5"))
    ax.annotate("", xy=(1.1, 0.45), xytext=(1.55, 0.95), arrowprops=dict(arrowstyle="-|>", color=PURPLE, lw=2, connectionstyle="arc3,rad=0.5"))
    ax.text(1.75, 1.55, "R", color=GREEN, fontsize=10, weight="bold"); ax.text(1.75, 0.5, "C", color=PURPLE, fontsize=10, weight="bold")
    ax.set_title("R / C = turn 90° clockwise / counter-\nclockwise (stay in the cell); W = wait", fontsize=9.5)
    ax = axes[2]; grid(ax); robot(ax, 0, 1, 0, label="A"); robot(ax, 2, 1, 2, color=ORANGE, label="B")
    ax.add_patch(mp.Rectangle((0.5, 0.5), 1, 1, color=RED, alpha=0.25))
    ax.text(1, 2.2, "both move F →\nsame cell: ✗", ha="center", fontsize=9, color=RED)
    ax.set_title("vertex conflict\n(two robots, one cell)", fontsize=9.5)
    ax = axes[3]; grid(ax); robot(ax, 0.5 - 0.5, 1, 0, label="A"); robot(ax, 1, 1, 2, color=ORANGE, label="B")
    ax.annotate("", xy=(0.95, 0.75), xytext=(0.05, 0.75), arrowprops=dict(arrowstyle="<|-|>", color=RED, lw=2))
    ax.text(1, 2.2, "A and B swap cells: ✗", ha="center", fontsize=9, color=RED)
    ax.set_title("edge (swap) conflict", fontsize=9.5)
    fig.suptitle("Action model (src/ActionModel.cpp): each tick every robot picks one action; the simulator rejects plans with conflicts",
                 fontsize=10.5, y=1.06)
    save(fig, "03_actions.png")


# --------------------------------------------------------------------- 4
def fig_task_life():
    rp = load(ROOT / "runs" / "distcheck" / "out" / "nms2024__pool1.5_kitchen_dist_d0_s0_T500.json")
    _index_events(rp)
    cand = [i for i, tk in rp.tasks.items() if tk["done"] > 0 and len(tk["locs"]) >= 3 and tk["reveal"] > 60]
    tid = sorted(cand, key=lambda i: rp.tasks[i]["done"] - rp.tasks[i]["reveal"])[len(cand) // 2]
    tk = rp.tasks[tid]
    holders = np.where((rp.assign == tid).any(axis=0))[0]
    a = int(holders[0])
    t_assign = int(np.where(rp.assign[:, a] == tid)[0][0])
    evs = sorted(tk["_ev"])
    fig, ax = plt.subplots(figsize=(12, 3.3))
    y = 0
    segs = [("waiting in pool\n(no robot yet)", tk["reveal"], t_assign, GREY)]
    prev = t_assign
    for k, (t, ag, _, seq) in enumerate(evs):
        loc = tk["locs"][seq - 1]
        on = [u for u in range(prev + 1, t + 1) if tuple(rp.pos[u, ag]) == tuple(loc)]
        arrive = on[0] if on else t
        segs.append((f"on the way to errand {seq}\n(moving or blocked)", prev, arrive, BLUE))
        segs.append((f"serve {seq}", arrive, t, ORANGE))
        prev = t
    for label, s0, s1, col in segs:
        if s1 <= s0:
            continue
        ax.barh(y, s1 - s0, left=s0, color=col, alpha=0.85, edgecolor="white", height=0.5)
        ax.text((s0 + s1) / 2, y + (0.45 if col == ORANGE else -0.45), label, ha="center", va="center", fontsize=8)
    ax.annotate("", xy=(tk["done"], -1.05), xytext=(tk["reveal"], -1.05), arrowprops=dict(arrowstyle="<|-|>", color="k"))
    ax.text((tk["reveal"] + tk["done"]) / 2, -1.3, f"latency = completed − revealed = {tk['done']} − {tk['reveal']} = {tk['done'] - tk['reveal']} ticks",
            ha="center", va="top", fontsize=9.5)
    ax.axvline(tk["reveal"], color="k", lw=0.8, ls=":"); ax.axvline(tk["done"], color="k", lw=0.8, ls=":")
    ax.set_yticks([]); ax.set_ylim(-1.9, 1.0); ax.set_xlabel("tick")
    ax.set_title(f"Life of one task (No Man's Sky, pool mode, kitchen service times): task {tid}, {len(tk['locs'])} errands, robot {a}",
                 fontsize=10)
    save(fig, "04_task_life.png")


# --------------------------------------------------------------------- 5
def fig_pool_vs_arrivals():
    fig, axes = plt.subplots(1, 2, figsize=(12, 3.4), gridspec_kw=dict(width_ratios=[1, 1.4]))
    js = json.loads((EXPLAIN / "nms2024__pool1.5_none_d0_s0_T1000.json").read_text())
    tt = np.array([[r[1], r[2]] for r in js["lorrExt"]["tasks"]])
    ts = np.arange(0, 1000)
    open_t = ((tt[:, 0][None] <= ts[:, None]) & ((tt[:, 1][None] < 0) | (tt[:, 1][None] > ts[:, None]))).sum(1)
    new_t = np.bincount(tt[:, 0].astype(int), minlength=1000)[:1000]
    ax = axes[0]
    ax.plot(ts, open_t, color=GREEN, label="open tasks (always 150)")
    ax.bar(ts[1:], new_t[1:] * 10, color=BLUE, alpha=0.5, width=1, label="new tasks this tick (×10; tick 0 reveals all 150)")
    ax.set_ylim(0, 200); ax.set_xlabel("tick"); ax.legend(fontsize=8, loc="lower right")
    ax.set_title("POOL (competition default): a finished task is\nreplaced immediately → 1.5 tasks per robot always open", fontsize=9.5)
    ax = axes[1]
    for m, col in (("poisson", BLUE), ("mmpp", RED), ("hawkes", PURPLE)):
        x = arrivals.sample(Setting(arrival=m, rate=0.3, seed=2), 600)
        w = np.convolve(x, np.ones(20), mode="same")
        ax.plot(np.arange(600), w, color=col, label=f"{m}: 20-tick moving sum")
    ax.axhline(0.3 * 20, color="k", ls=":", lw=0.8)
    ax.set_xlabel("tick"); ax.set_ylabel("new tasks per 20 ticks"); ax.legend(fontsize=8)
    ax.set_title("ARRIVAL PROCESSES (for benchmarking): same mean (0.3/tick, dotted),\ndifferent burstiness", fontsize=9.5)
    save(fig, "05_pool_vs_arrivals.png")


# --------------------------------------------------------------------- 6
def fig_service_dists():
    specs = [("10-30", "uniform interval 10–30"), ("fixed:20", "fixed 20"), ("normal:20:0.3", "normal, cv 0.3"),
             ("gamma:20:0.5", "gamma, cv 0.5"), ("lognormal:20:0.5", "lognormal, cv 0.5"), ("exponential:20", "exponential (cv 1)")]
    fig, axes = plt.subplots(2, 3, figsize=(12, 5.4), sharex=True)
    for ax, (spec, title) in zip(axes.flat, specs):
        x = arrivals.service_samples(service_stops(spec)[0], n=50000)
        ax.hist(x, bins=np.arange(0, 91) - 0.5, density=True, color=ORANGE, alpha=0.8)
        for q, ls in ((50, "-"), (95, "--")):
            v = np.percentile(x, q)
            ax.axvline(v, color="k", ls=ls, lw=1)
            ax.text(v + 1, ax.get_ylim()[1] * (0.85 if q == 50 else 0.65), f"p{q}={v:.0f}", fontsize=8)
        ax.set_title(f"{title}   `{spec}`", fontsize=9)
        ax.set_yticks([])
    for ax in axes[1]:
        ax.set_xlabel("service time (ticks)")
    fig.suptitle("Service-time options, all with mean 20 ticks. The scheduler knows the shape; each draw stays hidden.\n"
                 "Same mean, very different tails: p95 is what you must budget for a 95% guarantee.", fontsize=10.5)
    save(fig, "06_service_distributions.png")


# --------------------------------------------------------------------- 7
def fig_schedulers():
    robots = {"R1": (1, 1), "R2": (6, 3), "R3": (3, 5)}
    tasks = {"T1": ((2, 2), 5), "T2": ((7, 7), 60), "T3": ((5, 1), 10), "T4": ((1, 7), 30)}  # (first errand, age)
    path = {"T1": 60, "T2": 2, "T3": 4, "T4": 10}  # moves from first to last errand (+ p95 service)
    man = lambda a, b: abs(a[0] - b[0]) + abs(a[1] - b[1])
    def greedy():
        free, out = dict(tasks), {}
        for r, p in robots.items():  # each robot, in turn, takes its closest task
            t = min(free, key=lambda t: man(p, free[t][0])); out[r] = t; free.pop(t)
        return out
    def by_order(order):
        freer, out = dict(robots), {}
        for t in order:
            if not freer:
                break
            r = min(freer, key=lambda r: man(freer[r], tasks[t][0])); out[r] = t; freer.pop(r)
        return out
    oldest = sorted(tasks, key=lambda t: -tasks[t][1])
    D = 70
    slack = sorted(tasks, key=lambda t: D - tasks[t][1] - path[t])
    panels = [("greedy (2024/2026 default)\neach free robot takes its NEAREST task", greedy()),
              ("oldest-first (fifo)\noldest task gets the nearest free robot", by_order(oldest)),
              (f"least-slack (D = {D})\nsmallest  D − age − path − p95(service)  first", by_order(slack))]
    fig, axes = plt.subplots(1, 4, figsize=(15, 4))
    for ax, (title, asg) in zip(axes[:3], panels):
        ax.set_xlim(-0.5, 8.5); ax.set_ylim(8.5, -0.5); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
        for i in range(10):
            ax.axhline(i - 0.5, color="#eee"); ax.axvline(i - 0.5, color="#eee")
        for t, ((r, c), age) in tasks.items():
            col = plt.cm.autumn_r(min(age / 60, 1))
            ax.add_patch(mp.Rectangle((c - 0.4, r - 0.4), 0.8, 0.8, color=col))
            ax.text(c, r + 0.75, f"{t} age {age}\npath {path[t]}", ha="center", va="top", fontsize=7.5)
        for rb, (r, c) in robots.items():
            ax.add_patch(mp.Circle((c, r), 0.35, color=BLUE, zorder=3))
            ax.text(c, r, rb, color="white", ha="center", va="center", fontsize=7, weight="bold", zorder=4)
        for rb, t in asg.items():
            (r1, c1), ((r2, c2), _) = robots[rb], tasks[t]
            ax.annotate("", xy=(c2, r2), xytext=(c1, r1), arrowprops=dict(arrowstyle="-|>", color=GREEN, lw=2), zorder=2)
        left = [t for t in tasks if t not in asg.values()]
        ax.set_title(title + f"\n→ waits: {', '.join(left)} (age {tasks[left[0]][1]})", fontsize=9)
    ax = axes[3]
    ax.set_xlim(-0.5, 8.5); ax.set_ylim(8.5, -0.5); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    for i in range(10):
        ax.axhline(i - 0.5, color="#eee"); ax.axvline(i - 0.5, color="#eee")
    for (r, c) in [(2, 2), (5, 1), (7, 7)]:
        ax.add_patch(mp.Rectangle((c - 0.4, r - 0.4), 0.8, 0.8, color="#ffe8a3"))
    for (r, c), (r2, c2) in [((0, 5), (2, 2)), ((8, 3), (5, 1))]:
        ax.add_patch(mp.Circle((c, r), 0.35, color=GREY, zorder=3))
        ax.annotate("", xy=(c2, r2), xytext=(c, r), arrowprops=dict(arrowstyle="-|>", color=GREY, lw=2, ls="--"))
    ax.set_title("staging (idle robots only)\nidle robots drive to where tasks\nrecently started (yellow) and park", fontsize=9)
    fig.suptitle("Task assignment rules on a toy example (3 free robots, 4 open tasks; colour = task age)", fontsize=10.5)
    save(fig, "07_schedulers.png")


# --------------------------------------------------------------------- 8
def fig_pibt():
    fig, axes = plt.subplots(1, 3, figsize=(13, 2.9))
    def row(ax, cells, robots, title, notes=()):
        ax.set_xlim(-0.6, cells - 0.4); ax.set_ylim(-1.3, 1.0); ax.set_aspect("equal"); ax.axis("off")
        for i in range(cells):
            ax.add_patch(mp.Rectangle((i - 0.5, -0.5), 1, 1, fc="white", ec="#bbb"))
        for x, lab, col, pr in robots:
            ax.add_patch(mp.Circle((x, 0), 0.32, color=col, zorder=3))
            ax.text(x, 0, lab, color="white", ha="center", va="center", fontsize=8, weight="bold", zorder=4)
            ax.text(x, -0.75, f"prio {pr}", ha="center", fontsize=7.5)
        for a, b, col in notes:
            ax.annotate("", xy=(b, 0.55), xytext=(a, 0.55), arrowprops=dict(arrowstyle="-|>", color=col, lw=1.8,
                                                                         connectionstyle="arc3,rad=-0.4"))
        ax.set_title(title, fontsize=9)
    row(axes[0], 6, [(1, "A", BLUE, 9), (2, "B", BLUE, 3)],
        "1. A (high priority, waiting longest) wants\nB's cell: B INHERITS A's priority and must move", [(1, 2, RED), (2, 3, RED)])
    row(axes[1], 6, [(2, "A", BLUE, 9), (3, "B", BLUE, 9)],
        "2. B moves first (to its best free cell),\nthen A moves: no collision, nobody waits long", [])
    row(axes[2], 6, [(1, "A", BLUE, 9), (2, "I", GREY, 0)],
        "3. I = idle robot, lowest priority, facing the wrong\nway: it must TURN (1–2 ticks) before it can make room;\n"
        "some solvers never push it at all → A is stuck", [(1, 2, RED)])
    fig.suptitle("Planner core, PIBT (Priority Inheritance with Backtracking): every tick, robots in priority order pick their best next cell;"
                 "\na robot in the way inherits the priority and is pushed. If it can't move, the requester backtracks and tries another cell.",
                 fontsize=9.5)
    save(fig, "08_pibt.png")


# --------------------------------------------------------------------- 9
def fig_latency_metrics():
    from lorrext.core import latencies
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 3.8), sharey=True)
    for ax, (key, label, col) in zip(axes, [("nms2024", "No Man's Sky (2024 winner)", BLUE),
                                              ("default2026", "2026 default (greedy)", RED)]):
        js = json.loads((EXPLAIN / f"{key}__pool1.5_none_d0_s0_T1000.json").read_text())
        lat, cens = latencies(js)
        ax.hist(lat[~cens], bins=np.arange(0, 950, 20), color=col, alpha=0.7, label="finished tasks")
        ax.hist(lat[cens], bins=np.arange(0, 950, 20), color="k", alpha=0.5, label="still open at end (age so far)")
        worst = np.sort(lat)[-int(np.ceil(0.05 * len(lat))):]
        for (v, name, ls), hgt in zip(((np.percentile(lat, 50), "p50", ":"), (lat.mean(), "mean", "-"),
                                      (worst.mean(), "CVaR95", "-."), (np.percentile(lat, 99), "p99", "--")),
                                     (0.93, 0.78, 0.93, 0.78)):
            ax.axvline(v, color="k", ls=ls, lw=1.1)
            ax.text(v + 6, ax.get_ylim()[1] * hgt, f"{name}\n{v:.0f}", fontsize=7.5)
        ax.axvline(300, color=RED, lw=1.5)
        ax.text(305, ax.get_ylim()[1] * 0.5, "deadline\nD = 300", color=RED, fontsize=8)
        ax.set_title(f"{label}: throughput {js['numTaskFinished'] / 1000:.2f} tasks/tick", fontsize=9.5)
        ax.set_xlabel("latency (ticks)")
        ax.legend(fontsize=7.5, loc="center right")
    axes[0].set_ylabel("number of tasks")
    fig.suptitle("Latency distribution in pool mode: the mean says little; the tail (p99, CVaR95, misses past D) is where solvers differ",
                 fontsize=10.5)
    save(fig, "09_latency_metrics.png")


# --------------------------------------------------------------------- 10
def fig_capacity():
    df = pd.read_csv(ROOT / "results" / "runs.csv")
    pool = df[(df.scenario == "pool") & (df.solver == "nms2024")].throughput.mean()
    g = df[(df.scenario == "poisson") & (df.solver == "nms2024")].groupby("rate").miss_300
    m, sd, n = g.mean(), g.std(), g.size()
    half = 4.30 * sd / np.sqrt(n)  # 95% t-interval, 3 seeds
    fig, axes = plt.subplots(1, 2, figsize=(12.5, 3.9), gridspec_kw=dict(width_ratios=[1.6, 1]))
    ax = axes[0]
    ax.errorbar(m.index, m.values * 100, yerr=half.values * 100, fmt="o-", color=BLUE, capsize=3, label="No Man's Sky, mean ± 95% CI (3 seeds)")
    ax.axhline(1, color="k", ls=":", lw=1)
    ax.text(0.02, 1.4, "1% of tasks late  ⇔  p99 latency = deadline", fontsize=8)
    ax.axvline(0.7, color=GREEN, lw=1.2); ax.text(0.71, 6, "capacity\n= highest load\nwith ≤1% late", fontsize=8, color=GREEN)
    ax.axvspan(0, 0.15, color=RED, alpha=0.07); ax.text(0.01, 7.5, "light-load\nfailures", fontsize=8, color=RED)
    ax.set_xlabel("arrival rate λ (tasks/tick) = load"); ax.set_ylabel("% of tasks missing D = 300")
    ax.set_ylim(0, 9); ax.legend(fontsize=8, loc="upper center")
    ax.set_title("Deadline-feasible capacity: sweep the load, find the highest rate that keeps ≤1% late", fontsize=9.5)
    ax = axes[1]
    ax.bar([0], [pool], color=GREY, width=0.5); ax.bar([1], [0.7], color=GREEN, width=0.5)
    ax.set_xticks([0, 1], ["leaderboard\nthroughput\n(pool mode)", "deadline-feasible\ncapacity\n(D = 300)"])
    ax.annotate("", xy=(1, pool), xytext=(1, 0.7), arrowprops=dict(arrowstyle="<|-|>", color=RED))
    ax.text(1.28, (pool + 0.7) / 2, f"G = 1 − {0.7:.2f}/{pool:.2f}\n  = {1 - 0.7 / pool:.2f}", color=RED, fontsize=9, va="center")
    ax.set_ylabel("tasks/tick"); ax.set_xlim(-0.5, 2.0)
    ax.set_title("SLA gap G: share of leaderboard\nthroughput unusable under the deadline", fontsize=9.5)
    save(fig, "10_capacity_gap.png")


# --------------------------------------------------------------------- 11
def fig_littles_law():
    df = pd.read_csv(ROOT / "results" / "runs.csv")
    p = df[df.scenario == "pool"].groupby("solver")[["throughput", "lat_mean"]].mean()
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    x = np.linspace(0.35, 1.2, 50)
    ax.plot(x, 150 / x, color="k", lw=1, label="Little's law: W = L / λ = 150 / throughput")
    ax.scatter(p.throughput, p.lat_mean, color=BLUE, zorder=3, label="measured mean latency (1000 ticks)")
    for s, r in p.iterrows():
        ax.annotate(s.replace("2026", "").replace("2024", "'24"), (r.throughput, r.lat_mean), fontsize=7, xytext=(4, 2), textcoords="offset points")
    ax.set_xlabel("throughput λ (tasks/tick)"); ax.set_ylabel("mean latency W (ticks)")
    ax.legend(fontsize=7.5)
    ax.set_title("In pool mode L = 150 open tasks always, so the mean latency is\nfixed by throughput (measured values sit ~20% low because\ntasks still open at the end are counted at their age so far)", fontsize=9)
    save(fig, "11_littles_law.png")


if __name__ == "__main__":
    for f in (fig_system_loop, fig_world, fig_actions, fig_task_life, fig_pool_vs_arrivals, fig_service_dists,
              fig_schedulers, fig_pibt, fig_latency_metrics, fig_capacity, fig_littles_law):
        f()
