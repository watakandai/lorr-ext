"""Generate the figures used in docs/planning.md.

    uv run python docs/make_planning_figures.py

Most figures are schematic. Two are computed: the guide-path figures run the
2026 default planner's traffic-aware route cost on the real map, and the
action-mix figure reads the full-log pool runs in runs/explain/ (see the
explainer).
"""

from __future__ import annotations

import heapq
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mp
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "docs"))
from make_figures import BLUE, GREEN, GREY, ORANGE, PURPLE, RED, arrow, box, save  # noqa: E402

from lorrext import cellmaps  # noqa: E402
from lorrext.replay import load  # noqa: E402

MAP_FILE = ROOT / "instances" / "base" / "maps" / "random-32-32-20.map"
FREE = cellmaps.read_free(MAP_FILE)
EXPLAIN = ROOT / "runs" / "explain" / "out"
DR, DC = [0, 1, 0, -1], [1, 0, -1, 0]  # E S W N, as in the kit
ARROW = {0: (0.32, 0), 1: (0, -0.32), 2: (-0.32, 0), 3: (0, 0.32)}  # heading -> (dx, dy) in plot coords


# ----------------------------------------------------------------- drawing helpers
def grid(ax, rows, cols, walls=(), title="", fs=9.5):
    """Draw a rows x cols grid with (row, col) cells; row 0 at the top."""
    for r in range(rows):
        for c in range(cols):
            wall = (r, c) in walls
            ax.add_patch(mp.Rectangle((c, rows - 1 - r), 1, 1, fc="#343a40" if wall else "white", ec="#bbb", lw=1))
    ax.set_xlim(-0.1, cols + 0.1); ax.set_ylim(-0.1, rows + 0.1); ax.set_aspect("equal"); ax.axis("off")
    if title:
        ax.set_title(title, fontsize=fs)


def xy(rows, r, c):
    return c + 0.5, rows - 1 - r + 0.5


def robot(ax, rows, r, c, name, color=BLUE, heading=None, label=None, alpha=1.0):
    x, y = xy(rows, r, c)
    ax.add_patch(mp.Circle((x, y), 0.3, fc=color, ec="white", lw=1.5, alpha=alpha, zorder=3))
    ax.text(x, y, name, ha="center", va="center", color="white", fontsize=9, fontweight="bold", zorder=4)
    if heading is not None:
        dx, dy = ARROW[heading]
        ax.annotate("", xy=(x + dx * 1.45, y + dy * 1.45), xytext=(x + dx * 0.9, y + dy * 0.9),
                    arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6), zorder=4)
    if label:
        ax.text(x, y - 0.47, label, ha="center", va="top", fontsize=7.5, zorder=6,
                bbox=dict(fc="white", ec="none", pad=0.6, alpha=0.9))


def move(ax, rows, a, b, color=RED, rad=-0.35, ls="-"):
    ax.annotate("", xy=xy(rows, *b), xytext=xy(rows, *a),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.8, ls=ls, shrinkA=9, shrinkB=9,
                                connectionstyle=f"arc3,rad={rad}"), zorder=5)


def cross(ax, rows, r, c, color=RED):
    x, y = xy(rows, r, c)
    ax.plot([x - 0.25, x + 0.25], [y - 0.25, y + 0.25], color=color, lw=2.5, zorder=6)
    ax.plot([x - 0.25, x + 0.25], [y + 0.25, y - 0.25], color=color, lw=2.5, zorder=6)


# --------------------------------------------------------------------- p01
def fig_problem():
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.6))
    ax = axs[0]
    grid(ax, 3, 3, title="(a) A robot's state = cell + heading.\nEach tick it does ONE action", fs=9.5)
    robot(ax, 3, 1, 1, "A", heading=0)
    x, y = xy(3, 1, 1)
    ax.text(x + 0.98, y + 0.32, "FW", fontsize=8, color=BLUE, ha="center")
    ax.annotate("", xy=(x + 0.05, y + 0.48), xytext=(x + 0.48, y + 0.05),
                arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=1.4, connectionstyle="arc3,rad=0.4"))
    ax.text(x + 0.62, y + 0.62, "CCR", fontsize=8, color=ORANGE)
    ax.annotate("", xy=(x + 0.05, y - 0.48), xytext=(x + 0.48, y - 0.05),
                arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=1.4, connectionstyle="arc3,rad=-0.4"))
    ax.text(x + 0.62, y - 0.78, "CR", fontsize=8, color=ORANGE)
    ax.text(1.5, -0.45, "FW = forward 1 cell   CR / CCR = turn 90°   W = wait", ha="center", fontsize=8)

    ax = axs[1]
    grid(ax, 3, 3, title="(b) Vertex conflict: two robots\nin the same cell at the same tick")
    robot(ax, 3, 1, 0, "A", heading=0); robot(ax, 3, 0, 1, "B", color=PURPLE, heading=1)
    move(ax, 3, (1, 0), (1, 1), rad=0); move(ax, 3, (0, 1), (1, 1), rad=0, color=RED)
    cross(ax, 3, 1, 1)

    ax = axs[2]
    grid(ax, 3, 3, title="(c) Swap (edge) conflict: two robots\ntrade cells through each other")
    robot(ax, 3, 1, 0, "A", heading=0); robot(ax, 3, 1, 1, "B", color=PURPLE, heading=2)
    move(ax, 3, (1, 0), (1, 1), rad=-0.5); move(ax, 3, (1, 1), (1, 0), rad=-0.5)
    x, y = xy(3, 1, 0)
    ax.plot([x + 0.5], [y], "x", color=RED, ms=14, mew=3, zorder=6)
    fig.suptitle("The planner's job each tick: pick one action per robot so that no two robots collide, "
                 "and everyone gets closer to their goal", fontsize=10.5, y=1.03)
    save(fig, "p01_planning_problem.png")


# --------------------------------------------------------------------- p02
def fig_space_time():
    fig, axs = plt.subplots(1, 3, figsize=(12.5, 3.9), gridspec_kw=dict(width_ratios=[1.1, 1, 1]))
    ax = axs[0]
    walls = [(0, c) for c in range(7) if c != 3]
    grid(ax, 2, 7, walls=walls, title="A corridor with one side bay.\nA goes right, B goes left")
    robot(ax, 2, 1, 0, "A", heading=0); robot(ax, 2, 1, 6, "B", color=PURPLE, heading=2)
    x, y = xy(2, 0, 3)
    ax.text(x, y, "bay", ha="center", va="center", fontsize=8, color="#555")
    for c in range(7):
        ax.text(c + 0.5, -0.32, str(c), ha="center", fontsize=8, color="#555")

    t = np.arange(0, 9)
    BAY = 7.3
    for k, ax in enumerate(axs[1:]):
        if k == 0:
            a, b = np.minimum(t, 6), np.maximum(6 - t, 0)
            ax.plot(t, a, "-o", color=BLUE, label="A"); ax.plot(t, b, "-o", color=PURPLE, label="B")
            ax.plot([3], [3], "o", ms=26, mfc="none", mec=RED, mew=2)
            ax.text(4.0, 2.9, "both in cell 3\nat t = 3: collision", color=RED, fontsize=8, ha="left", va="center")
            ax.set_title("Space-time view: each line is one robot.\nLines meet = conflict", fontsize=9.5)
        else:
            a = [0, 1, 2, 2, 3, 4, 5, 6, 6]
            b = [6, 5, 4, 3, BAY, BAY, 3, 2, 1]
            ax.plot(t, a, "-o", color=BLUE, label="A")
            ax.plot(t, b, "-o", color=PURPLE, label="B")
            ax.axhline(BAY - 0.65, color="#ccc", lw=0.8)
            ax.annotate("A waits 1 tick", xy=(3, 2), xytext=(3.6, 0.6), fontsize=8, color=BLUE,
                        arrowprops=dict(arrowstyle="-", color=BLUE))
            ax.text(5.6, BAY, "B waits in the bay", fontsize=8, color=PURPLE, ha="left", va="center")
            ax.set_title("Fix: B ducks into the bay, A waits one tick\nand passes; flat line = waiting", fontsize=9.5)
        ax.set_xlabel("time t (ticks)"); ax.set_ylabel("corridor cell")
        ax.set_xlim(-0.3, 8.5); ax.set_ylim(-0.4, 7.9)
        ax.set_yticks(list(range(7)) + ([BAY] if k else []), [str(i) for i in range(7)] + (["bay"] if k else []))
        ax.legend(fontsize=8, loc="center right" if k else "lower left")
    save(fig, "p02_space_time.png")


# --------------------------------------------------------------------- p03
def fig_algorithm_map():
    fig, ax = plt.subplots(figsize=(10, 5.4))
    # (x = how far ahead, y = how hard it works to coordinate, label, color, used here?)
    pts = [
        (1, 1.0, "PIBT\n(one-step, greedy)", GREY, False),
        (11, 1.4, "2026 default\nPIBT repeated 11× + traffic guide paths", BLUE, True),
        (5, 2.6, "No Man's Sky ('24 1st)\n5-action moves, 32 PIBT tries + annealing", GREEN, True),
        (20, 3.0, "Kitty_Knight ('24 2nd, WPPL)\n20-step window, LNS repair", ORANGE, True),
        (60, 2.5, "LaCAM\ncomplete search\nover whole-team moves", PURPLE, False),
        (60, 3.6, "LaCAM*\n+ keeps improving\n(eventually optimal)", PURPLE, False),
        (60, 4.4, "CBS\noptimal, small teams only", GREY, False),
        (20, 1.75, "Prioritized planning + SIPP\none robot at a time", GREY, False),
    ]
    for x, y, lab, c, used in pts:
        ax.scatter([x], [y], s=240 if used else 150, color=c, ec="k" if used else "none", lw=1.5, zorder=3,
                   marker="o" if used else "s")
        ha = "left"
        dx = 1.12
        if x == 60:
            ha, dx = "right", 0.88
        ax.text(x * dx, y, lab, fontsize=8, va="center", ha=ha)
    ax.set_xscale("log")
    ax.set_xticks([1, 5, 11, 20, 60], ["1 tick", "5", "11", "20", "whole path"])
    ax.set_xlim(0.7, 130); ax.set_ylim(0.5, 4.9)
    ax.set_yticks([1, 2.5, 4], ["greedy:\nfirst answer", "repair /\noptimise", "complete /\noptimal"])
    ax.set_xlabel("how far ahead each robot's moves are planned (ticks, log scale)")
    ax.set_ylabel("how hard it searches for a\ngood joint move (qualitative)")
    ax.scatter([], [], s=150, color="w", ec="k", lw=1.5, label="in our benchmark")
    ax.scatter([], [], s=110, color=GREY, marker="s", label="background / Kei18 candidates")
    ax.legend(fontsize=8, loc="lower right")
    ax.set_title("A map of the planners in this document. Up and right = more computation per tick;\n"
                 "LoRR gives very little time per tick, so winners sit low and left", fontsize=10)
    save(fig, "p03_algorithm_map.png")


# --------------------------------------------------------------------- p04
def fig_timeline():
    fig, ax = plt.subplots(figsize=(12, 3.9))
    lanes = {"planner\n(own thread)": 2.4, "executor": 1.3, "simulator\n(ticks)": 0.3}
    for lab, y in lanes.items():
        ax.text(-25, y + 0.3, lab, ha="right", va="center", fontsize=9)
    cols = [BLUE, GREEN, ORANGE]
    for k in range(3):
        s = k * 300
        ax.add_patch(mp.Rectangle((s + 4, 2.4), 292, 0.6, fc=cols[k], alpha=0.25, ec=cols[k]))
        ax.text(s + 150, 2.7, f"plan #{k + 1}: 11 steps\nfrom the predicted state at t = {10 * (k + 1)}",
                ha="center", va="center", fontsize=8)
        if k < 2:
            ax.annotate("", xy=(s + 300, 1.9), xytext=(s + 296, 2.4),
                        arrowprops=dict(arrowstyle="-|>", color=cols[k], lw=1.5))
            ax.add_patch(mp.Rectangle((s + 300, 1.3), 300, 0.6, fc=cols[k], alpha=0.12, ec=cols[k]))
            ax.text(s + 450, 1.6, f"stages plan #{k + 1} (≤ 11 actions/robot)\nGO / STOP to each robot every tick",
                    ha="center", va="center", fontsize=8)
    for t in range(0, 901, 30):
        ax.add_patch(mp.Rectangle((t + 2, 0.3), 26, 0.6, fc="#dee2e6", ec="#adb5bd", lw=0.6))
    ax.text(15, 0.6, "1", ha="center", va="center", fontsize=6.5)
    ax.text(315, 0.6, "11", ha="center", va="center", fontsize=6.5)
    ax.annotate("1 tick = 30 ms, whether or not the planner is done", xy=(615, 0.95), xytext=(640, 1.15),
                fontsize=8, arrowprops=dict(arrowstyle="-", color="#555"))
    ax.set_xlim(-130, 920); ax.set_ylim(0.1, 3.4)
    ax.set_xticks(range(0, 901, 150)); ax.set_xlabel("wall-clock time (ms)")
    ax.set_yticks([]); ax.spines["left"].set_visible(False)
    ax.set_title("2026 kit: the planner runs beside the simulation. Every 300 ms it hands over a plan of 300/30 + 1 = 11 steps;\n"
                 "the executor feeds it to the robots tick by tick while the planner already works on the next one", fontsize=9.5)
    save(fig, "p04_timeline_2026.png")


# --------------------------------------------------------------------- p05
def fig_pibt():
    rows, cols = 3, 4
    walls = [(0, 0), (0, 2), (0, 3), (1, 3), (2, 0), (2, 1), (2, 2), (2, 3)]
    fig, axs = plt.subplots(1, 4, figsize=(13, 3.9))
    titles = ["1. A (highest priority) wants B's\ncell → B INHERITS A's priority",
              "2. B's best cell is C's → C inherits.\nC has no free cell → C FAILS",
              "3. B BACKTRACKS: tries its next-best\ncell (up) → free → success",
              "4. Result for this tick: A and B move,\nC waits. No collisions"]
    for k, ax in enumerate(axs):
        grid(ax, rows, cols, walls=walls, title=titles[k], fs=9)
        if k < 3:
            robot(ax, rows, 1, 0, "A", label="prio 0.9")
            robot(ax, rows, 1, 1, "B", color=PURPLE, label="0.9" if k else "0.4")
            robot(ax, rows, 1, 2, "C", color=GREEN, label="0.9" if k in (1, 2) else "0.2")
        if k == 0:
            move(ax, rows, (1, 0), (1, 1))
        if k == 1:
            move(ax, rows, (1, 1), (1, 2))
            cross(ax, rows, 1, 2, color=RED)
        if k == 2:
            move(ax, rows, (1, 1), (0, 1), color=GREEN, rad=0.3)
            move(ax, rows, (1, 1), (1, 2), color=GREY, ls=":")
        if k == 3:
            robot(ax, rows, 1, 1, "A"); robot(ax, rows, 0, 1, "B", color=PURPLE); robot(ax, rows, 1, 2, "C", color=GREEN)
    fig.text(0.5, 0.06, "Rules that keep it safe: a robot never moves into the cell its 'parent' is leaving "
             "(no swaps), and a cell, once claimed this tick, is taken.\nEach robot ranks cells by its "
             "distance-to-goal heuristic h; ties prefer the cell straight ahead (no turn needed).",
             ha="center", fontsize=8.5)
    save(fig, "p05_pibt_backtracking.png")


# --------------------------------------------------------------------- p06
def fig_rotation():
    fig, axs = plt.subplots(1, 3, figsize=(12.5, 3.8))
    ax = axs[0]
    grid(ax, 3, 3, title="(a) PIBT picks CELLS. Facing east but\nwanting north: CCR now, FW next tick")
    robot(ax, 3, 1, 1, "A", heading=0)
    move(ax, 3, (1, 1), (0, 1), color=BLUE, rad=0)
    ax.text(1.5, -0.4, "turning costs ticks the plan didn't count", ha="center", fontsize=8)
    ax = axs[1]
    grid(ax, 3, 3, title="(b) Wanting the cell behind:\nCR, CR, then FW (3 ticks)")
    robot(ax, 3, 1, 1, "A", heading=0)
    move(ax, 3, (1, 1), (1, 0), color=BLUE, rad=0)
    ax = axs[2]
    grid(ax, 1, 5, title="(c) moveCheck: a forward move happens only if\nthe robot in front really moves this tick")
    robot(ax, 1, 0, 1, "A", heading=0); robot(ax, 1, 0, 2, "B", color=PURPLE, heading=0)
    robot(ax, 1, 0, 3, "C", color=GREEN, heading=3)
    for c, txt, col in [(1, "FW→W", RED), (2, "FW→W", RED), (3, "turning", GREEN)]:
        ax.text(c + 0.5, -0.3, txt, ha="center", fontsize=8.5, color=col)
    ax.text(2.5, -0.75, "C turns instead of moving, so B and A must wait too", ha="center", fontsize=8)
    ax.set_ylim(-1.0, 1.3)
    fig.text(0.5, -0.05, "The default planner ignores turns while planning, then converts each chosen cell into an action "
             "(getAction) and fixes chains of blocked forward moves (moveCheck).", ha="center", fontsize=8.5)
    save(fig, "p06_turns_and_movecheck.png")


# --------------------------------------------------------------------- p07/p08 (computed)
def _route(start, goal, flow, aware):
    """Dijkstra over free cells. aware=True uses the kit's guide-path cost
    (search.cpp): 1 + (flow[u][d]+1)*flow[v][opposite d] + sum(flow[v])//2 per step."""
    rows, cols = FREE.shape
    dist = {start: 0}
    par = {start: None}
    pq = [(0, 0, start)]
    tick = 0
    while pq:
        g, _, u = heapq.heappop(pq)
        if u == goal:
            break
        if g > dist[u]:
            continue
        for d in range(4):
            v = (u[0] + DR[d], u[1] + DC[d])
            if not (0 <= v[0] < rows and 0 <= v[1] < cols and FREE[v]):
                continue
            step = 1
            if aware:
                step += (flow[u][d] + 1) * flow[v][(d + 2) % 4] + int(flow[v].sum()) // 2
            ng = g + step
            if ng < dist.get(v, 1e18):
                dist[v] = ng
                par[v] = u
                tick += 1
                heapq.heappush(pq, (ng, tick, v))
    path, u = [], goal
    while u is not None:
        path.append(u)
        u = par[u]
    return path[::-1]


def _add(flow, path, sign):
    for u, v in zip(path, path[1:]):
        d = [k for k in range(4) if (u[0] + DR[k], u[1] + DC[k]) == v][0]
        flow[u][d] += sign


def _stats(paths, flow):
    rows, cols = FREE.shape
    use = np.zeros(FREE.shape)
    for p in paths:
        for c in p[1:]:
            use[c] += 1
    headon = 0
    for r in range(rows):
        for c in range(cols):
            for d in (0, 1):  # count each undirected edge once
                v = (r + DR[d], c + DC[d])
                if v[0] < rows and v[1] < cols and FREE[r, c] and FREE[v]:
                    headon += flow[(r, c)][d] * flow[v][(d + 2) % 4]
    return use, headon, np.mean([len(p) - 1 for p in paths])


def guide_paths(n=100, passes=4, seed=0):
    rng = np.random.default_rng(seed)
    cells = [(int(r), int(c)) for r, c in np.argwhere(FREE)]
    pick = rng.choice(len(cells), size=2 * n, replace=False)
    pairs = [(cells[pick[i]], cells[pick[n + i]]) for i in range(n)]
    flow = {c: np.zeros(4, dtype=int) for c in cells}
    paths = [_route(s, g, flow, aware=False) for s, g in pairs]
    for p in paths:
        _add(flow, p, +1)
    before = _stats(paths, flow)
    plain = [list(p) for p in paths]
    for _ in range(passes):  # re-route one robot at a time against everyone else (as frank_wolfe does)
        for i, (s, g) in enumerate(pairs):
            _add(flow, paths[i], -1)
            paths[i] = _route(s, g, flow, aware=True)
            _add(flow, paths[i], +1)
    after = _stats(paths, flow)
    return pairs, plain, paths, before, after


def fig_guide_paths(data):
    pairs, plain, guided, (u0, h0, l0), (u1, h1, l1) = data
    fig, axs = plt.subplots(1, 2, figsize=(11.5, 5.6))
    vmax = max(u0.max(), u1.max())
    for ax, use, title in [(axs[0], u0, f"Shortest paths\nhead-on pairs = {h0}, mean length {l0:.1f}"),
                           (axs[1], u1, f"Traffic-aware guide paths\nhead-on pairs = {h1}, mean length {l1:.1f}")]:
        img = np.where(FREE, use, np.nan)
        cmap = plt.get_cmap("magma_r").copy(); cmap.set_bad("#343a40")
        im = ax.imshow(img, cmap=cmap, vmin=0, vmax=vmax)
        ax.set_title(title, fontsize=9.5); ax.set_xticks([]); ax.set_yticks([])
    fig.colorbar(im, ax=axs, shrink=0.75, label="how many of the 100 guide paths pass through the cell")
    fig.suptitle("Traffic-flow guidance on the benchmark map (100 robots, random start → goal). Re-routing each robot "
                 "against\neveryone else's flow trades a slightly longer route for far fewer robots meeting head-on "
                 "in a corridor", fontsize=10)
    save(fig, "p07_guide_paths.png")


def _bfs(src):
    rows, cols = FREE.shape
    dist = np.full(FREE.shape, np.inf)
    dist[src] = 0
    q = [src]
    for u in q:
        for d in range(4):
            v = (u[0] + DR[d], u[1] + DC[d])
            if 0 <= v[0] < rows and 0 <= v[1] < cols and FREE[v] and dist[v] == np.inf:
                dist[v] = dist[u] + 1
                q.append(v)
    return dist


def fig_guide_heuristic(data):
    pairs, plain, guided, *_ = data
    i = int(np.argmax([len(g) - len(p) for g, p in zip(guided, plain)]))  # the robot rerouted the most
    path = guided[i]
    start, goal = pairs[i]
    h_goal = _bfs(goal)
    togo = {c: len(path) - 1 - k for k, c in enumerate(path)}
    # h(v) = distance from v to the nearest guide-path cell + steps left along the path from there
    h_gp = np.full(FREE.shape, np.inf)
    for c, rem in togo.items():
        h_gp = np.minimum(h_gp, _bfs(c) + rem)
    fig, axs = plt.subplots(1, 2, figsize=(11.5, 5.4))
    for ax, h, title in [(axs[0], h_goal, "h = plain distance to goal\n(downhill = straight to the goal)"),
                         (axs[1], h_gp, "h = distance to the guide path + steps left along it\n"
                                        "(downhill = back onto the guide path, then along it)")]:
        img = np.where(FREE, h, np.nan)
        cmap = plt.get_cmap("viridis").copy(); cmap.set_bad("#343a40")
        ax.imshow(img, cmap=cmap)
        X, Y, U, V = [], [], [], []
        for r, c in np.argwhere(FREE):
            nb = [(h[r + DR[d], c + DC[d]], d) for d in range(4)
                  if 0 <= r + DR[d] < FREE.shape[0] and 0 <= c + DC[d] < FREE.shape[1] and FREE[r + DR[d], c + DC[d]]]
            if nb and min(nb)[0] < h[r, c]:
                d = min(nb)[1]
                X.append(c); Y.append(r); U.append(DC[d]); V.append(DR[d])
        ax.quiver(X, Y, U, V, angles="xy", scale_units="xy", scale=2.4, color="w", alpha=0.55, width=0.0035)
        pr, pc = zip(*path)
        ax.plot(pc, pr, color="w", lw=2.2)
        sp = plain[i]
        if ax is axs[0]:
            qr, qc = zip(*sp)
            ax.plot(qc, qr, color="w", lw=1.4, ls="--")
            ax.text(0.02, -0.06, "dashed = shortest path, solid = guide path", transform=ax.transAxes, fontsize=8)
        ax.plot(start[1], start[0], "o", color=GREEN, ms=9, mec="w")
        ax.plot(goal[1], goal[0], "*", color=RED, ms=15, mec="w")
        ax.set_title(title, fontsize=9.5); ax.set_xticks([]); ax.set_yticks([])
    fig.suptitle("What PIBT ranks cells by (one robot; green = robot, red star = goal). "
                 "Dark = small h; small arrows = the neighbour PIBT tries first", fontsize=10)
    save(fig, "p08_guide_heuristic.png")


# --------------------------------------------------------------------- p09
def fig_episode():
    fig, ax = plt.subplots(figsize=(12, 3.3))
    T = 290  # 300 ms minus the 10 ms tolerance
    pibt = 100  # max(MIN_PIBT_TIME = 100, 1 ms per 100 robots x 11 steps)
    flow = T - pibt - 10
    ax.add_patch(mp.Rectangle((0, 1.2), 25, 0.7, fc=GREY, alpha=0.4))
    ax.text(12, 1.55, "setup", ha="center", va="center", fontsize=8)
    ax.add_patch(mp.Rectangle((25, 1.2), flow - 25, 0.7, fc=BLUE, alpha=0.3))
    ax.text((25 + flow) / 2, 1.55, "update guide paths of robots with a new goal,\nthen re-route others "
            "against traffic (frank_wolfe) until time is up", ha="center", va="center", fontsize=8)
    ax.add_patch(mp.Rectangle((flow + 10, 1.2), T - flow - 10, 0.7, fc=ORANGE, alpha=0.3))
    ax.text((flow + 10 + T) / 2, 1.55, "PIBT × 11 steps", ha="center", va="center", fontsize=8)
    w = (T - flow - 10) / 11
    for k in range(11):
        x0 = flow + 10 + k * w
        ax.add_patch(mp.Rectangle((x0 + 1, 0.25), w - 2, 0.55, fc=ORANGE, alpha=0.55))
        ax.text(x0 + w / 2, 0.52, str(k + 1), ha="center", va="center", fontsize=7)
    ax.annotate("each step: run PIBT on the IMAGINED positions after the\nprevious step, record the actions, "
                "roll the copy forward", xy=(flow + 10, 0.5), xytext=(20, 0.45), fontsize=8, va="center",
                arrowprops=dict(arrowstyle="-|>", color="#555"))
    ax.set_xlim(-5, 300); ax.set_ylim(0, 2.3)
    ax.set_xlabel("time inside one 300 ms planning call (ms)"); ax.set_yticks([]); ax.spines["left"].set_visible(False)
    ax.set_title("2026 default planner, one call (planner.cpp plan): most of the budget goes to traffic-aware guide "
                 "paths;\nPIBT is fast, so 11 rollouts fit in the last ~100 ms. The real positions are restored afterwards",
                 fontsize=9.5)
    save(fig, "p09_default_planner_call.png")


# --------------------------------------------------------------------- p10
def fig_executor():
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.9), gridspec_kw=dict(width_ratios=[1, 1, 1.1]))
    ax = axs[0]
    grid(ax, 3, 3, walls=[(0, 0), (0, 2), (2, 0), (2, 2)],
         title="Plan: A crosses X at t=1,\nB enters X at t=2 (right behind A)")
    robot(ax, 3, 1, 0, "A", heading=0); robot(ax, 3, 0, 1, "B", color=PURPLE, heading=1)
    x, y = xy(3, 1, 1)
    ax.text(x, y, "X", ha="center", va="center", fontsize=12, color="#555")
    move(ax, 3, (1, 0), (1, 2), color=BLUE, rad=-0.25); move(ax, 3, (0, 1), (2, 1), color=PURPLE, rad=0.25)
    ax = axs[1]
    ax.axis("off"); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.set_title("Executor keeps, for every cell, the ORDER\nin which robots are planned to enter it", fontsize=9.5)
    box(ax, (0.12, 0.5), 0.76, 0.22, "cell X:   A  →  B", GREEN, fs=11)
    ax.text(0.5, 0.3, "B gets GO into X only when B is first\nin X's queue, i.e. after A has left X", ha="center",
            fontsize=9)
    ax = axs[2]
    t = np.arange(0, 6)
    a_plan = [0, 1, 2, 2, 2, 2]
    a_real = [0, 0, 0, 1, 2, 2]  # A is delayed 2 ticks
    b_real = [0, 0, 0, 0, 1, 2]
    ax.step(t, a_plan, where="post", color=BLUE, ls=":", label="A planned")
    ax.step(t, a_real, where="post", color=BLUE, lw=2, label="A actual (delayed 2 ticks)")
    ax.step(t + 0.04, b_real, where="post", color=PURPLE, lw=2, label="B actual (STOP until A clears X)")
    ax.set_yticks([0, 1, 2], ["before X", "in X", "past X"]); ax.set_xlabel("tick")
    ax.axvspan(1, 3, color=RED, alpha=0.08); ax.text(2, 2.15, "delay", color=RED, ha="center", fontsize=8)
    ax.legend(fontsize=7.5, loc="upper left"); ax.set_ylim(-0.3, 2.6)
    ax.set_title("With delays, robots keep the planned order\ninstead of the planned timing → no collision",
                 fontsize=9.5)
    save(fig, "p10_executor_tpg.png")


# --------------------------------------------------------------------- p11
def fig_nms():
    fig, axs = plt.subplots(1, 3, figsize=(13.5, 4.3), gridspec_kw=dict(width_ratios=[1.1, 1, 1]))
    ax = axs[0]
    rows, cols = 5, 7
    grid(ax, rows, cols, title="(a) Each robot chooses a 5-action 'operation'\n(129 precomputed, written like FFFFF or "
                               "RFFWW; R / C = turn right / left)")
    robot(ax, rows, 2, 1, "A", heading=0)
    ops = [([(2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6)], BLUE, "FFFFF"),
           ([(2, 1), (1, 1), (0, 1)], GREEN, "CCR F F W W"),
           ([(2, 1), (2, 2), (3, 2), (4, 2)], ORANGE, "F CR F F W")]
    for k, (cells, c, lab) in enumerate(ops):
        pts = [xy(rows, *p) for p in cells]
        px, py = zip(*pts)
        off = (k - 1) * 0.07
        ax.plot(np.array(px) + off, np.array(py) + off, color=c, lw=2.2, alpha=0.85)
        ax.text(px[-1] + 0.1, py[-1] + 0.25, lab, color=c, fontsize=8)
    ax = axs[1]
    ax.axis("off"); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.set_title("(b) Operations reserve (cell, step 1–5) and edges;\nPIBT-style: in priority order, take the "
                 "best\noperation that is still free, push lower robots", fontsize=9)
    hdr = ["cell", "s1", "s2", "s3", "s4", "s5"]
    rowsd = [["(2,2)", "A", "", "", "", ""], ["(2,3)", "", "A", "", "B", ""], ["(2,4)", "", "", "A", "", ""]]
    for j, h in enumerate(hdr):
        ax.text(0.1 + j * 0.15, 0.75, h, ha="center", fontsize=9, fontweight="bold")
    for i, rw in enumerate(rowsd):
        for j, v in enumerate(rw):
            fc = {"A": BLUE, "B": PURPLE}.get(v)
            if fc:
                ax.add_patch(mp.Rectangle((0.1 + j * 0.15 - 0.06, 0.62 - i * 0.14 - 0.05), 0.12, 0.1, fc=fc, alpha=0.3))
            ax.text(0.1 + j * 0.15, 0.62 - i * 0.14, v, ha="center", va="center", fontsize=9)
    ax.text(0.5, 0.12, "score = Σ robots (distance-to-goal gained)\n× robot weight", ha="center", fontsize=8.5)
    ax = axs[2]
    rng = np.random.default_rng(3)
    s = np.sort(rng.normal(100, 6, 32))[::-1]
    ax.bar(range(32), s, color=[GREEN] + [GREY] * 31)
    ax.set_ylim(80, 128); ax.set_xlabel("32 threads (different random seeds)"); ax.set_ylabel("score (schematic)")
    ax.text(1.5, s[0] + 1.0, "best → its FIRST action\nis sent; repeat next tick", fontsize=8, color=GREEN)
    ax.set_title("(c) Each thread: greedy start, then simulated\nannealing (re-pick one robot's operation,\n"
                 "accept worse with small probability)", fontsize=9)
    save(fig, "p11_nms_pibts.png")


# --------------------------------------------------------------------- p12
def fig_wppl():
    fig, axs = plt.subplots(1, 3, figsize=(14.5, 4.3), gridspec_kw=dict(width_ratios=[1, 1.6, 0.9]))
    ax = axs[0]
    rng = np.random.default_rng(1)
    for i in range(6):
        y = np.cumsum(rng.choice([-1, 0, 1], size=21, p=[0.2, 0.3, 0.5])) + i * 4
        ax.plot(range(21), y, color=plt.cm.tab10(i), lw=1.6)
        ax.plot([0, 1], y[:2], color=plt.cm.tab10(i), lw=4)
    ax.axvspan(0, 1, color=GREEN, alpha=0.15)
    ax.text(1.3, ax.get_ylim()[1] - 1, "← executed: 1 step", color=GREEN, fontsize=8, va="top")
    ax.set_xlabel("steps ahead"); ax.set_yticks([]); ax.set_xlim(0, 20)
    ax.set_title("(a) A collision-free plan for EVERY robot\nover a 20-step window; only step 1 runs,\n"
                 "then the window slides", fontsize=9)
    ax = axs[1]
    ax.axis("off"); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.set_title("(b) Large Neighbourhood Search (LNS) loop", fontsize=9.5)
    box(ax, (0.3, 0.8), 0.4, 0.13, "start: LaCAM2 window plan", PURPLE, fs=8.5)
    box(ax, (0.0, 0.45), 0.42, 0.2, "DESTROY: pick 8 robots\n(random / random walk /\nnear an intersection)", RED, fs=7.5)
    box(ax, (0.58, 0.45), 0.42, 0.2, "REPAIR: re-plan them one\nby one around everyone\nelse (prioritized\nplanning + SIPP)",
        BLUE, fs=7.5)
    box(ax, (0.29, 0.07), 0.42, 0.17, "KEEP if total path cost\nwent down, else undo", GREEN, fs=8)
    arrow(ax, (0.5, 0.8), (0.3, 0.66))
    arrow(ax, (0.42, 0.55), (0.58, 0.55))
    arrow(ax, (0.79, 0.45), (0.62, 0.24))
    arrow(ax, (0.38, 0.24), (0.21, 0.45), text="repeat until\ntime is up", fs=7.5, off=(-0.1, 0))
    ax = axs[2]
    it = np.arange(0, 400)
    soc = 1600 + 380 * np.exp(-it / 90) + rng.normal(0, 4, it.size)
    ax.plot(it, np.minimum.accumulate(soc), color=BLUE)
    ax.set_xlabel("LNS iteration"); ax.set_ylabel("sum of path costs (schematic)")
    ax.set_title("(c) Each kept repair makes the\nwhole plan a bit shorter", fontsize=9)
    save(fig, "p12_wppl_lns.png")


# --------------------------------------------------------------------- p13
def fig_sipp():
    fig, axs = plt.subplots(1, 2, figsize=(12, 3.4), gridspec_kw=dict(width_ratios=[1.3, 1]))
    ax = axs[0]
    busy = [(3, 4), (7, 9)]
    for a, b in busy:
        ax.add_patch(mp.Rectangle((a, 0.3), b - a, 0.4, fc=GREY, alpha=0.7))
        ax.text((a + b) / 2, 0.5, "busy", ha="center", va="center", fontsize=8, color="w")
    for a, b, lab in [(0, 3, "[0, 2]"), (4, 7, "[4, 6]"), (9, 14, "[9, ∞)")]:
        ax.add_patch(mp.Rectangle((a, 0.3), b - a, 0.4, fc=GREEN, alpha=0.25, ec=GREEN))
        ax.text((a + b) / 2, 0.85, f"safe {lab}", ha="center", fontsize=8.5, color=GREEN)
    ax.set_xlim(0, 14); ax.set_ylim(0, 1.15); ax.set_yticks([]); ax.set_xlabel("time t")
    ax.set_title("One cell over time: other robots pass at t = 3 and t = 7–8 (grey).\nWhat's left is a few SAFE INTERVALS (green)", fontsize=9.5)
    ax = axs[1]
    ax.axis("off"); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    ax.text(0.02, 0.85, "Space-time A*: state = (cell, t) → huge", fontsize=9.5)
    ax.text(0.02, 0.65, "SIPP: state = (cell, safe interval) → small", fontsize=9.5, color=GREEN)
    ax.text(0.02, 0.3, "Arrive as EARLY as possible inside each interval;\nwaiting is free while the interval lasts. "
            "Same\nanswer as space-time A*, far fewer states.", fontsize=8.5)
    save(fig, "p13_sipp.png")


# --------------------------------------------------------------------- p14
def fig_lacam():
    fig, ax = plt.subplots(figsize=(11.5, 4.6))
    ax.axis("off"); ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    box(ax, (0.03, 0.62), 0.2, 0.2, "configuration Q0\n(where EVERY robot\nis now)", PURPLE, fs=8.5)
    box(ax, (0.4, 0.78), 0.22, 0.16, "Q1 = PIBT(Q0)\nno constraints", BLUE, fs=8.5)
    box(ax, (0.4, 0.46), 0.22, 0.2, "Q1' = PIBT(Q0) with\n'robot 1 must go up'", BLUE, fs=8.5)
    box(ax, (0.4, 0.14), 0.22, 0.2, "Q1'' = PIBT(Q0) with\n'robot 1 up AND\nrobot 2 waits'", BLUE, fs=8.5)
    arrow(ax, (0.23, 0.74), (0.4, 0.86), text="1st visit", fs=7.5)
    arrow(ax, (0.23, 0.70), (0.4, 0.56), text="2nd visit", fs=7.5)
    arrow(ax, (0.23, 0.66), (0.4, 0.25), text="3rd visit", fs=7.5)
    box(ax, (0.72, 0.62), 0.25, 0.3, "every configuration Q is a\nnode in a search tree\n→ DFS until everyone\n"
                                     "is at their goal", GREEN, fs=8.5)
    box(ax, (0.72, 0.14), 0.25, 0.32, "LaCAM*: keep searching after\nthe first answer and rewire\nto cheaper parents "
                                      "→ the\nsolution improves over time\n(eventually optimal)", ORANGE, fs=8.5)
    ax.text(0.03, 0.3, "Each time the search comes back to Q0\nit LAZILY adds one more constraint, so\n"
            "PIBT is forced to try a different move.\nThis makes LaCAM complete: it finds a\nsolution "
            "whenever one exists.", fontsize=8.5, va="center")
    ax.set_title("LaCAM (Okumura 2023): a search over whole-team moves, using PIBT as the move generator",
                 fontsize=10.5)
    save(fig, "p14_lacam.png")


# --------------------------------------------------------------------- p15 (empirical)
def fig_action_mix():
    runs = [("2026 default", "default2026"), ("oldest-first (2026)", "fifo2026"), ("No Man's Sky (2024 kit)", "nms2024")]
    labels, fw, turn, wait = [], [], [], []
    for lab, s in runs:
        f = EXPLAIN / f"{s}__pool1.5_none_d0_s0_T1000.json"
        if not f.exists():
            print("skip action mix: missing", f)
            return
        rp = load(f, MAP_FILE)
        moved = np.any(rp.pos[1:] != rp.pos[:-1], axis=2)
        turned = (rp.ori[1:] != rp.ori[:-1]) & ~moved
        n = moved.size
        labels.append(lab); fw.append(moved.sum() / n); turn.append(turned.sum() / n)
        wait.append(1 - (moved.sum() + turned.sum()) / n)
    fig, ax = plt.subplots(figsize=(8.5, 3.4))
    y = np.arange(len(labels))
    ax.barh(y, fw, color=BLUE, label="moving forward")
    ax.barh(y, turn, left=fw, color=ORANGE, label="turning")
    ax.barh(y, wait, left=np.add(fw, turn), color=GREY, label="waiting")
    for i in range(len(labels)):
        ax.text(fw[i] / 2, i, f"{fw[i]:.0%}", ha="center", va="center", color="w", fontsize=9)
        ax.text(fw[i] + turn[i] / 2, i, f"{turn[i]:.0%}", ha="center", va="center", fontsize=9)
        ax.text(fw[i] + turn[i] + wait[i] / 2, i, f"{wait[i]:.0%}", ha="center", va="center", fontsize=9)
    ax.set_yticks(y, labels); ax.invert_yaxis(); ax.set_xlim(0, 1)
    ax.set_xlabel("share of robot-ticks (pool mode, 100 robots, 1000 ticks, seed 0)")
    ax.legend(fontsize=8, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.0))
    save(fig, "p15_action_mix.png")
    return dict(zip(labels, zip(fw, turn, wait)))


if __name__ == "__main__":
    for f in (fig_problem, fig_space_time, fig_algorithm_map, fig_timeline, fig_pibt, fig_rotation,
              fig_episode, fig_executor, fig_nms, fig_wppl, fig_sipp, fig_lacam):
        f()
    d = guide_paths()
    print("guide paths: head-on before/after", d[3][1], d[4][1], "mean length", round(d[3][2], 2), round(d[4][2], 2))
    fig_guide_paths(d)
    fig_guide_heuristic(d)
    print("action mix", fig_action_mix())
