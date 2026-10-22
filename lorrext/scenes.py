"""Scenes: small hand-made situations that show how planners and schedulers differ.

Each scene is a tiny map plus robots and a script of tasks, built to bring out
one behaviour (a head-on meeting in a corridor, a far-away task that greedy
never picks, ...). Every relevant solver runs on it with the *real* kit
binaries, and the result is drawn side by side.

    python -m lorrext.scenes list
    python -m lorrext.scenes run corridor_swap            # run + render one scene
    python -m lorrext.scenes run --all                     # every scene (parallel, pinned)
    python -m lorrext.scenes render corridor_swap          # re-render from cached runs
    python -m lorrext.scenes hunt runs/explain/out/X.json  # find the worst jam in a big run

Outputs go to docs/scenes/: <scene>.gif (animation), <scene>_snap.png (rows =
solvers, columns = key ticks) and <scene>.json (metrics per solver).

Notes
* The 2024 entries were written for the five competition maps. On a scene map
  No Man's Sky runs without its hand-drawn lane map for random-32-32-20 (and
  with its scheduler's distance weight 1 instead of 5). Kitty_Knight is not
  used in scenes (see PLANNERS); `hunt` shows it on the benchmark map.
* Tasks use the "scripted" arrival model: task i appears at tick times[i].
"""

from __future__ import annotations

import argparse
import json
import queue
import threading
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import core
from .replay import DC, DR, Replay, _index_events, load

ROOT = core.ROOT
RUNS = ROOT / "runs" / "scenes"
DOCS = ROOT / "docs" / "scenes"
STUCK_TICKS = 4  # a busy robot that hasn't changed cell for this many ticks is drawn as stuck


# ----------------------------------------------------------------- scene definitions
@dataclass
class Run:
    solver: str
    label: str = ""
    delay: dict | None = None      # per-cell delay map override (2026 kit only)

    def key(self) -> str:
        return self.solver + ("_delay" if self.delay else "")


@dataclass
class Scene:
    name: str
    kind: str                      # "planner" | "scheduler"
    title: str
    lesson: str
    grid: list[str]                # '.' free, '@' wall
    robots: list[tuple[int, int]]
    tasks: list[list[tuple[int, int]]]
    times: list[int]               # reveal tick of each task (scripted arrivals)
    runs: list[Run]
    steps: int = 150
    deadline: int = 300
    service: dict | None = None    # per-cell service map, cellmaps JSON format
    frames: list[int] = field(default_factory=list)  # snapshot ticks (default: evenly spaced)

    @property
    def shape(self):
        return len(self.grid), len(self.grid[0])

    def free(self) -> np.ndarray:
        return np.array([[ch == "." for ch in row] for row in self.grid])


PLANNERS = [Run("default2026", "2026 default\n(guided PIBT, 11-step plans)"),
            Run("default2024", "2024 default\n(guided PIBT, 1 step)"),
            Run("nms2024", "No Man's Sky\n(PIBT over 5-step moves + annealing)")]
# Kitty_Knight is left out of the scenes: it assumes tasks never run out, and on these finite
# scenes it stalls and then crashes soon after the first task is finished. It appears in `hunt`
# clips from the 100-robot benchmark runs instead.
SCHED_2026 = [Run("default2026", "greedy (2026 default)\nnearest task per robot"),
              Run("fifo2026", "oldest-first\noldest task, nearest robot"),
              Run("slack2026", "least-slack\nmost urgent task first")]


def _own_tasks(robots, goals):
    """One task per robot whose first errand is the robot's own cell. No Man's Sky then gives
    robot i task i; the greedy defaults minimise total task cost and may swap tasks between
    robots, but the 2024 and 2026 defaults make the same swaps, so they still see the same jobs."""
    return [[r, g] for r, g in zip(robots, goals)]


def corridor_swap() -> Scene:
    grid = ["@@@@@@@@@@@@@@@@@@@@@",
            "@.....@@@@.@@@@.....@",
            "@.....@@@@.@@@@.....@",
            "@...................@",
            "@.....@@@@@@@@@.....@",
            "@.....@@@@@@@@@.....@",
            "@@@@@@@@@@@@@@@@@@@@@"]
    left = [(1, 4), (3, 4), (5, 4)]
    right = [(1, 16), (3, 16), (5, 16)]
    robots = left + right
    goals = [(1, 18), (3, 18), (5, 18), (1, 2), (3, 2), (5, 2)]
    return Scene("corridor_swap", "planner", "Head-on in a corridor with one passing bay",
                 "Three robots go right and three go left through a one-lane corridor. The only place to "
                 "pass is a two-cell bay above the middle. A planner must make one side wait in the bay "
                 "or back out; a greedy one-step rule tends to push robots back and forth.",
                 grid, robots, _own_tasks(robots, goals), [0] * 6, PLANNERS, steps=120,
                 frames=[0, 12, 25, 45, 80])


def two_doors() -> Scene:
    grid = ["@@@@@@@@@@@@@@@@@",
            "@.......@.......@",
            "@...............@",
            "@.......@.......@",
            "@.......@.......@",
            "@.......@.......@",
            "@.......@.......@",
            "@.......@.......@",
            "@...............@",
            "@.......@.......@",
            "@@@@@@@@@@@@@@@@@"]
    left = [(1, 2), (2, 2), (3, 2), (1, 4), (2, 4), (3, 4)]
    right = [(1, 14), (2, 14), (3, 14), (1, 12), (2, 12), (3, 12)]
    robots = left + right
    goals_all = [(1, 13), (2, 13), (3, 13), (1, 15), (2, 15), (3, 15),
                 (1, 3), (2, 3), (3, 3), (1, 1), (2, 1), (3, 1)]
    return Scene("two_doors", "planner", "Two teams, two doors",
                 "Six robots cross left to right and six right to left. Everyone's shortest path uses the "
                 "top door; the bottom door is a long detour. Traffic-aware guide paths should send one "
                 "team the long way instead of meeting head-on in the top door. (No Man's Sky leaves two robots "
                 "unfinished: robots with no task stand still, and two of them park on others' goal cells; "
                 "its planner never pushes an idle robot aside because in the competition no robot is idle.)",
                 grid, robots, _own_tasks(robots, goals_all), [0] * 12, PLANNERS, steps=150,
                 frames=[0, 10, 25, 50, 100])


def idle_blocker() -> Scene:
    grid = ["@@@@@@@@@@@@@@@@@@@",
            "@...@@@@@.@@@@@...@",
            "@.................@",
            "@...@@@@@@@@@@@...@",
            "@@@@@@@@@@@@@@@@@@@"]
    robots = [(3, 1), (2, 9)]
    tasks = [[(3, 1), (2, 17)],  # robot 0: across the corridor
             [(2, 9)]]           # robot 1: done on the spot, then idle in the corridor
    # greedy schedulers visit free robots from the highest index down, so robot 1 picks first
    return Scene("idle_blocker", "planner", "An idle robot parked in a corridor",
                 "Robot 1 has nothing to do and sits in the middle of a one-lane corridor. Robot 0 needs to "
                 "get through. Does the planner move the idle robot out of the way (there is a one-cell "
                 "pocket right above it), and where does the idle robot go afterwards?",
                 grid, robots, tasks, [0, 0], PLANNERS, steps=70, frames=[0, 8, 16, 24, 50])


def delay_convoy() -> Scene:
    grid = ["@@@@@@@@@@@@@@@@@@@@@@",
            "@....................@",
            "@@@@@@@@@@@@@@@@@@@@@@"]
    robots = [(1, c) for c in (5, 4, 3, 2, 1)]
    goals = [(1, c) for c in (20, 19, 18, 17, 16)]
    slow = {"default": None, "cells": [{"name": "sticky cell", "cells": [[1, 9]], "p": 0.6,
                                         "dist": "uniform", "lo": 4, "hi": 6, "trigger": "enter"}]}
    return Scene("delay_convoy", "planner", "A convoy meets a sticky cell (2026 executor)",
                 "Five robots drive in a line. On the right run, one cell makes a robot stall 4–6 ticks "
                 "on 60% of its visits. The plan doesn't know; the executor keeps the planned "
                 "order, so everyone behind the stalled robot gets STOP and waits, without collisions.",
                 grid, robots, _own_tasks(robots, goals), [0] * 5,
                 [Run("default2026", "2026 default, no delays"),
                  Run("default2026", "2026 default, sticky cell (p = 0.6)", delay=slow)],
                 steps=90, frames=[0, 24, 32, 51, 70])


def crowded_room() -> Scene:
    h, w = 10, 12
    grid = ["@" * w] + ["@" + "." * (w - 2) + "@" for _ in range(h - 2)] + ["@" * w]
    grid[4] = "@....@@....@"
    grid[5] = "@....@@....@"
    rng = np.random.default_rng(3)
    cells = [(r, c) for r in range(h) for c in range(w) if grid[r][c] == "."]
    pick = rng.choice(len(cells), size=24, replace=False)
    robots = [cells[i] for i in pick]
    tasks = [[cells[i], cells[j]] for i, j in rng.integers(0, len(cells), size=(400, 2))]
    times = sorted(rng.integers(0, 200, size=400).tolist())
    return Scene("crowded_room", "planner", "A crowded room",
                 "24 robots in a small room with a pillar, 2 new tasks per tick on average. No clever "
                 "assignment is possible; the planner decides how much of the time robots move, turn or "
                 "wait. Watch the red rings (stuck robots).",
                 grid, robots, tasks, times, PLANNERS, steps=200, frames=[20, 60, 120, 190])


def far_task() -> Scene:
    grid = ["@" * 26] + ["@" + "." * 24 + "@" for _ in range(7)] + ["@" * 26]
    robots = [(2, 2), (4, 2), (6, 2), (3, 4)]
    near = [(r, c) for r in range(1, 8) for c in range(1, 8)]
    rng = np.random.default_rng(1)
    tasks = [[(4, 23), (2, 24)]]  # the far task, revealed first
    times = [0]
    for t in range(1, 200, 2):     # a steady stream of short tasks next to the robots
        a, b = rng.choice(len(near), size=2, replace=False)
        tasks.append([near[a], near[b]])
        times.append(t)
    return Scene("far_task", "scheduler", "The far-away task",
                 "One task appears at the far end at t = 0; short tasks keep appearing right next to the "
                 "robots (faster than they can serve them). Who ever goes to the far task?",
                 grid, robots, tasks, times, SCHED_2026 + [Run("nms2024", "No Man's Sky\n(annealing, may re-assign)")],
                 steps=200, frames=[0, 30, 100, 199])


def scattered_batch() -> Scene:
    grid = ["@" * 22] + ["@" + "." * 20 + "@" for _ in range(10)] + ["@" * 22]
    robots = [(2, 2), (2, 19), (9, 2), (9, 19)]
    rng = np.random.default_rng(5)
    cells = [(r, c) for r in range(1, 11) for c in range(1, 21)]
    tasks = [[cells[i], cells[j]] for i, j in rng.integers(0, len(cells), size=(16, 2))]
    return Scene("scattered_batch", "scheduler", "A batch of scattered tasks",
                 "16 tasks appear at once all over the map, in random order. Oldest-first serves them in "
                 "that order, so robots criss-cross the map; greedy takes whatever is nearest. Compare the "
                 "trails and the finish time.",
                 grid, robots, tasks, [0] * 16, SCHED_2026, steps=150, frames=[0, 20, 50, 100])


def slow_station() -> Scene:
    grid = ["@" * 22] + ["@" + "." * 20 + "@" for _ in range(7)] + ["@" * 22]
    robots = [(4, 1), (4, 2), (3, 1)]
    station = [(1, 18), (7, 18)]
    tasks, times = [], []
    for k in range(4):  # 2-errand tasks that end at a slow station (40 ticks of work)
        tasks.append([(4, 12), station[k % 2]])
        times.append(0)
    for k in range(9):  # quick tasks close to the robots
        tasks.append([(1 + k % 7, 3 + k % 3), (1 + (k + 3) % 7, 5)])
        times.append(0)
    svc = {"default": None, "cells": [{"name": "slow station", "cells": [list(c) for c in station],
                                       "dist": "fixed", "mean": 40}]}
    return Scene("slow_station", "scheduler", "Urgent first, everything late",
                 "Four tasks end at a slow station (40 ticks of work); nine quick tasks sit next to the "
                 "robots. Deadline D = 90 ticks, and three robots cannot finish everything in time. "
                 "Least-slack knows the stations are slow and sends robots there first; the four slow jobs "
                 "queue at two stations and every quick task misses the deadline. Greedy does the quick "
                 "tasks first and sacrifices the slow ones: fewer late tasks. Under overload, urgent-first "
                 "spreads lateness instead of containing it. (Oldest-first matches least-slack here only "
                 "because all tasks arrive at once and ties go to the lowest id, the slow tasks.)",
                 grid, robots, tasks, times, SCHED_2026, steps=200, deadline=90, service=svc,
                 frames=[0, 25, 60, 120])


def reassign() -> Scene:
    grid = ["@" * 26] + ["@" + "." * 24 + "@" for _ in range(5)] + ["@" * 26]
    robots = [(3, 1), (3, 20)]
    tasks = [[(3, 20), (1, 20), (5, 20)],  # robot 1's own task: keeps it busy ~10 ticks nearby
             [(5, 24), (1, 24)]]           # the new task, close to robot 1 but robot 0 is the only free one
    runs = [Run("default2024", "2024 default\n(greedy, commits)"),
            Run("nms2024", "No Man's Sky\n(re-optimises every tick)"),
            Run("default2026", "2026 default\n(greedy, commits)")]
    return Scene("reassign", "scheduler", "Changing your mind: re-assignment",
                 "Robot 1 is busy right next to a new task; robot 0 is free but far away. A committing "
                 "scheduler sends robot 0 on the long trip. One that may re-assign unopened tasks can "
                 "hand the task to robot 1 as soon as it is free.",
                 grid, robots, tasks, [0, 0], runs, steps=60, frames=[0, 6, 14, 24, 40])


SCENES = {f().name: f for f in (corridor_swap, two_doors, idle_blocker, delay_convoy, crowded_room,
                                 far_task, scattered_batch, slow_station, reassign)}


# ----------------------------------------------------------------- writing + running
def _cell(sc: Scene, rc) -> int:
    return rc[0] * sc.shape[1] + rc[1]


def write_files(sc: Scene, run: Run) -> Path:
    d = RUNS / sc.name
    for sub in ("maps", "agents", "tasks", "out"):
        (d / sub).mkdir(parents=True, exist_ok=True)
    h, w = sc.shape
    mname = f"scene-{sc.name}"
    (d / "maps" / f"{mname}.map").write_text(f"type octile\nheight {h}\nwidth {w}\nmap\n" + "\n".join(sc.grid) + "\n")
    (d / "agents" / f"{sc.name}.agents").write_text(
        f"# version for LoRR 2024\n{len(sc.robots)}\n" + "\n".join(str(_cell(sc, r)) for r in sc.robots) + "\n")
    (d / "tasks" / f"{sc.name}.tasks").write_text(
        f"# version for LoRR 2024\n{len(sc.tasks)}\n" + "\n".join(",".join(str(_cell(sc, c)) for c in t) for t in sc.tasks) + "\n")
    _kitty_files(sc, mname)
    inst = {
        "mapFile": f"maps/{mname}.map", "agentFile": f"agents/{sc.name}.agents", "teamSize": len(sc.robots),
        "taskFile": f"tasks/{sc.name}.tasks", "numTasksReveal": 1.0, "agentCounter": 1, "agentSize": 1.0,
        "delayConfig": {"seed": 0, "minDelay": 1, "maxDelay": 4, "eventModel": "bernoulli", "pDelay": 0.0,
                        "durationModel": "uniform", "gaussMeanRatio": 0.5, "gaussStdRatio": 0.22},
        "lorrExt": {"arrival": {"model": "scripted", "times": sc.times, "seed": 0},
                    "mapCols": w,
                    "service": {**(sc.service or {}), "seed": 0},
                    "delay": {**(run.delay or {}), "seed": 0},
                    "deadline": sc.deadline, "scheduler": core.SOLVERS[run.solver]["scheduler"]},
    }
    p = d / f"{run.key()}.json"
    p.write_text(json.dumps(inst, indent=1))
    return p


def _kitty_files(sc: Scene, mname: str) -> None:
    """Kitty_Knight loads wppl_planner/configs/<map>.json and data/map_stats/<map>.json."""
    kit = core.KITS / "kitty2024"
    cfg = json.loads((kit / "wppl_planner" / "configs" / "random-32-32-20.json").read_text())
    # LNS destroys neighborSize robots at a time (8); it loops forever looking for 8 in a smaller team
    cfg["LNS"]["neighborSize"] = max(2, min(8, len(sc.robots) - 1))
    (kit / "wppl_planner" / "configs" / f"{mname}.json").write_text(json.dumps(cfg, indent=1))
    stats = kit / "data" / "map_stats" / f"{mname}.json"
    stats.write_text(json.dumps({"num_empty_locs": int(sc.free().sum()), "num_S_locs": 0, "num_E_locs": 0,
                                 "S_locs": [], "E_locs": []}))


def out_path(sc: Scene, run: Run) -> Path:
    return RUNS / sc.name / "out" / f"{run.key()}.json"


def run_scene(sc: Scene, force: bool = False, cpus: str | None = None) -> None:
    for run in sc.runs:
        out = out_path(sc, run)
        if out.exists() and not force:
            continue
        inst = write_files(sc, run)
        rc, err = core.run_kit(run.solver, inst, out, sc.steps, screen=1, timeout=900, cpus=cpus)
        print(f"{sc.name:16s} {run.key():18s} rc={rc}" + ("" if out.exists() else "  NO OUTPUT\n" + err[-800:]))


def run_all(names: list[str], force: bool, cores_per_run: int = 2, workers: int = 6) -> None:
    jobs: queue.Queue = queue.Queue()
    for n in names:
        sc = SCENES[n]()
        for run in sc.runs:
            jobs.put((sc, run))
    slots: queue.Queue = queue.Queue()
    for w in range(workers):
        slots.put(",".join(str(w * cores_per_run + c) for c in range(cores_per_run)))

    def worker():
        while True:
            try:
                sc, run = jobs.get_nowait()
            except queue.Empty:
                return
            cpus = slots.get()
            try:
                one = Scene(**{**sc.__dict__, "runs": [run]})
                run_scene(one, force, cpus)
            finally:
                slots.put(cpus)

    ts = [threading.Thread(target=worker) for _ in range(workers)]
    for t in ts:
        t.start()
    for t in ts:
        t.join()


# ----------------------------------------------------------------- analysis
def stuck_mask(rp: Replay) -> np.ndarray:
    """(T+1, A) True where a busy robot has stayed on the same cell for >= STUCK_TICKS ticks."""
    same = np.all(rp.pos[1:] == rp.pos[:-1], axis=2)
    run = np.zeros(rp.pos.shape[:2], dtype=np.int32)
    for t in range(1, rp.pos.shape[0]):
        run[t] = np.where(same[t - 1], run[t - 1] + 1, 0)
    return (run >= STUCK_TICKS) & (rp.assign >= 0)


def next_errand(rp: Replay, tid: int, t: int):
    tk = rp.tasks[tid]
    k = sum(1 for e in tk["_ev"] if e[0] <= t)
    return tk["locs"][min(k, len(tk["locs"]) - 1)] if k < len(tk["locs"]) else None


def metrics(rp: Replay, deadline: int) -> dict:
    T = rp.T
    lat = [tk["done"] - tk["reveal"] for tk in rp.tasks.values() if tk["done"] >= 0]
    waits = [(tk["done"] if tk["done"] >= 0 else T) - tk["reveal"] for tk in rp.tasks.values() if tk["reveal"] <= T]
    moved = np.any(rp.pos[1:] != rp.pos[:-1], axis=2)
    robots_per_task = {}
    for t in range(T + 1):
        for a, tid in enumerate(rp.assign[t]):
            if tid >= 0:
                robots_per_task.setdefault(int(tid), set()).add(a)
    return {"done": len(lat), "revealed": len(waits),
            "mean_latency": round(float(np.mean(lat)), 1) if lat else None,
            "max_wait": int(max(waits)) if waits else 0,
            "late": int(sum(w > deadline for w in waits)),
            "stuck_robot_ticks": int(stuck_mask(rp).sum()),
            "move_share": round(float(moved.mean()), 3),
            "reassigned_tasks": int(sum(len(v) > 1 for v in robots_per_task.values())),
            "last_done": int(max((tk["done"] for tk in rp.tasks.values()), default=-1))}


# ----------------------------------------------------------------- drawing
STATE_COLORS = ["#9aa0a6", "#1c7ed6", "#f08c00"]  # idle, travelling, at errand


def _state(rp: Replay, a: int, t: int) -> int:
    tid = rp.assign[t, a]
    if tid < 0 or tid not in rp.tasks:
        return 0
    nxt = next_errand(rp, tid, t)
    return 2 if nxt is not None and tuple(rp.pos[t, a]) == tuple(nxt) else 1


def draw(ax, rp: Replay, t: int, stuck: np.ndarray, deadline: int, title: str | None = None, trail: int = 10):
    import matplotlib.patches as mp
    from matplotlib.colors import LinearSegmentedColormap

    age_cmap = LinearSegmentedColormap.from_list("age", ["#ffe066", "#e8590c", "#c92a2a"])
    ax.clear()
    ax.imshow(rp.obstacles, cmap="Greys", vmin=0, vmax=1.25, interpolation="nearest")
    h, w = rp.obstacles.shape
    for x in range(w + 1):
        ax.axvline(x - 0.5, color="#e9ecef", lw=0.5, zorder=0.5)
    for y in range(h + 1):
        ax.axhline(y - 0.5, color="#e9ecef", lw=0.5, zorder=0.5)
    # open tasks: square on the next errand, coloured by age; outline = past the deadline
    for tid, tk in rp.tasks.items():
        if not (tk["reveal"] <= t and (tk["done"] < 0 or tk["done"] > t)):
            continue
        nxt = next_errand(rp, tid, t)
        if nxt is None:
            continue
        age = t - tk["reveal"]
        late = age > deadline
        ax.add_patch(mp.Rectangle((nxt[1] - 0.42, nxt[0] - 0.42), 0.84, 0.84, fc=age_cmap(min(age / deadline, 1)),
                                  ec="k" if late else "none", lw=1.6 if late else 0, alpha=0.9, zorder=1))
    A = rp.pos.shape[1]
    for a in range(A):
        r, c = rp.pos[t, a]
        st = _state(rp, a, t)
        col = STATE_COLORS[st]
        t0 = max(0, t - trail)
        ax.plot(rp.pos[t0:t + 1, a, 1], rp.pos[t0:t + 1, a, 0], color=col, lw=1.2, alpha=0.35, zorder=2)
        tid = rp.assign[t, a]
        if tid >= 0 and tid in rp.tasks:
            nxt = next_errand(rp, tid, t)
            if nxt is not None and tuple(nxt) != (r, c):
                ax.plot([c, nxt[1]], [r, nxt[0]], color=col, lw=0.9, ls="--", alpha=0.8, zorder=2)
        ax.add_patch(mp.Circle((c, r), 0.36, fc=col, ec="#e03131" if stuck[t, a] else "white",
                               lw=2.2 if stuck[t, a] else 0.8, zorder=3))
        o = rp.ori[t, a]
        ax.add_patch(mp.Polygon([(c + DC[o] * 0.5, r + DR[o] * 0.5),
                                 (c + DC[o] * 0.22 - DR[o] * 0.2, r + DR[o] * 0.22 + DC[o] * 0.2),
                                 (c + DC[o] * 0.22 + DR[o] * 0.2, r + DR[o] * 0.22 - DC[o] * 0.2)],
                                fc="k", ec="none", zorder=4))
        if A <= 12:
            ax.text(c - 0.05 * DC[o], r - 0.05 * DR[o], str(a), ha="center", va="center", fontsize=6.5,
                    color="white", zorder=5)
    done = sum(1 for tk in rp.tasks.values() if 0 <= tk["done"] <= t)
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_xlim(-0.5, w - 0.5); ax.set_ylim(h - 0.5, -0.5)
    head = (title if title is not None else rp.label)
    ax.set_title(f"{head}\nt = {t}   done {done}   stuck {int(stuck[t].sum())}", fontsize=8.5)


LEGEND = ("robots: grey = idle, blue = going to a task, orange = working at an errand; red ring = stuck "
          f"(busy, same cell for ≥ {STUCK_TICKS} ticks); dashed = robot → its next errand; "
          "squares = open tasks (yellow → red with age, black outline = past the deadline)")


def _load(sc: Scene):
    path = RUNS / sc.name / "maps" / f"scene-{sc.name}.map"
    reps = []
    for run in sc.runs:
        out = out_path(sc, run)
        if not out.exists():
            print("missing run:", out)
            continue
        rp = load(out, path, run.label or core.SOLVERS[run.solver]["label"])
        _index_events(rp)
        reps.append(rp)
    return reps


def render(sc: Scene, gif: bool = True) -> dict:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    reps = _load(sc)
    if not reps:
        return {}
    DOCS.mkdir(parents=True, exist_ok=True)
    stucks = [stuck_mask(rp) for rp in reps]
    T = min(rp.T for rp in reps)
    h, w = sc.shape
    n = len(reps)
    cell = 0.32 if w > 14 else 0.42

    # --- snapshot grid: rows = solvers, columns = key ticks
    frames = [f for f in (sc.frames or np.linspace(0, T, 4).astype(int).tolist()) if f <= T]
    fig, axs = plt.subplots(n, len(frames), figsize=(len(frames) * (w * cell + 0.3), n * (h * cell + 0.9)),
                            squeeze=False)
    for i, rp in enumerate(reps):
        for j, t in enumerate(frames):
            draw(axs[i, j], rp, t, stucks[i], sc.deadline, title=rp.label.replace("\n", " ") if j == 0 else "")
    fig.suptitle(f"{sc.title}\n{_wrap(sc.lesson, 150)}", fontsize=10, y=1.0)
    fig.text(0.5, -0.01, _wrap(LEGEND, 170), ha="center", va="top", fontsize=7.5)
    fig.tight_layout()
    fig.savefig(DOCS / f"{sc.name}_snap.png", dpi=110, bbox_inches="tight")
    plt.close(fig)

    # --- animation: solvers side by side + cumulative tasks done
    if gif:
        cols = min(n, 2 if w > 14 else 4)
        rows = (n + cols - 1) // cols
        fig = plt.figure(figsize=(cols * (w * cell + 0.4), rows * (h * cell + 1.0) + 1.6), dpi=80)
        gs = fig.add_gridspec(rows + 1, cols, height_ratios=[h * cell + 1.0] * rows + [1.3], hspace=0.35)
        axs = [fig.add_subplot(gs[k // cols, k % cols]) for k in range(n)]
        strip = fig.add_subplot(gs[rows, :])
        palette = ["#1c7ed6", "#e03131", "#2f9e44", "#ae3ec9"]
        cum = [np.array([sum(1 for tk in rp.tasks.values() if 0 <= tk["done"] <= t) for t in range(T + 1)])
               for rp in reps]
        stride = -(-T // 100)  # at most ~100 frames: keeps the GIFs small
        imgs = []
        for t in range(0, T + 1, stride):
            for k, rp in enumerate(reps):
                draw(axs[k], rp, t, stucks[k], sc.deadline)
            strip.clear()
            for k, rp in enumerate(reps):
                strip.plot(np.arange(t + 1), cum[k][:t + 1], color=palette[k % 4], lw=1.5,
                           label=rp.label.split("\n")[0])
            strip.set_xlim(0, T); strip.set_ylim(0, max(c.max() for c in cum) + 1)
            strip.set_ylabel("tasks done", fontsize=8); strip.tick_params(labelsize=7)
            strip.legend(fontsize=7, loc="upper left", ncol=min(n, 4))
            fig.suptitle(sc.title, fontsize=10)
            fig.canvas.draw()
            imgs.append(Image.frombuffer("RGBA", fig.canvas.get_width_height(), fig.canvas.buffer_rgba()).convert(
                "P", palette=Image.Palette.ADAPTIVE, colors=64))
        imgs[0].save(DOCS / f"{sc.name}.gif", save_all=True, append_images=imgs[1:], duration=110 * stride ** 0.5,
                     loop=0, optimize=True)
        plt.close(fig)

    res = {rp.label.replace("\n", " "): metrics(rp, sc.deadline) for rp in reps}
    (DOCS / f"{sc.name}.json").write_text(json.dumps(res, indent=1))
    return res


def _wrap(s: str, width: int) -> str:
    import textwrap

    return "\n".join(textwrap.wrap(s, width))


# ----------------------------------------------------------------- hunting in big runs
def hunt(out_json: Path, window: int = 20, label: str = "", pad: int = 3, box: int = 8, warmup: int = 50) -> Path:
    """Find the densest jam (most stuck robots in one box x box area) in a full-log run, after a
    warm-up, and render a zoomed clip around it."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image

    rp = load(out_json, label=label or out_json.stem.split("__")[0])
    _index_events(rp)
    stuck = stuck_mask(rp)
    # stuck robots per box x box area at each tick, via a summed-area table over the grid
    dense = np.zeros((rp.T + 1, rp.rows - box + 1, rp.cols - box + 1), dtype=np.int32)
    for t in range(warmup, rp.T + 1):
        who = np.where(stuck[t])[0]
        if len(who) == 0:
            continue
        g = np.zeros((rp.rows + 1, rp.cols + 1), dtype=np.int32)
        np.add.at(g, (rp.pos[t, who, 0] + 1, rp.pos[t, who, 1] + 1), 1)
        g = g.cumsum(0).cumsum(1)
        dense[t] = g[box:, box:] - g[:-box, box:] - g[box:, :-box] + g[:-box, :-box]
    # summed over a short window (a jam, not a blip)
    score = np.apply_along_axis(lambda x: np.convolve(x, np.ones(10), mode="same"), 0, dense)
    t_star, br, bc = (int(i) for i in np.unravel_index(np.argmax(score), score.shape))
    inbox = stuck[t_star] & (rp.pos[t_star, :, 0] >= br) & (rp.pos[t_star, :, 0] < br + box) \
        & (rp.pos[t_star, :, 1] >= bc) & (rp.pos[t_star, :, 1] < bc + box)
    who = np.where(inbox)[0]
    r0, r1 = max(0, br - pad), min(rp.rows, br + box + pad)
    c0, c1 = max(0, bc - pad), min(rp.cols, bc + box + pad)
    DOCS.mkdir(parents=True, exist_ok=True)
    name = f"hunt_{out_json.stem.split('__')[0]}"
    fig, ax = plt.subplots(figsize=(6, 6 * (r1 - r0) / max(1, c1 - c0) + 0.8), dpi=90)
    imgs = []
    ts = range(max(0, t_star - window), min(rp.T, t_star + window) + 1)
    for t in ts:
        draw(ax, rp, t, stuck, 300, title=f"{rp.label}: worst jam around t = {t_star}")
        ax.set_xlim(c0 - 0.5, c1 - 0.5); ax.set_ylim(r1 - 0.5, r0 - 0.5)
        fig.canvas.draw()
        imgs.append(Image.frombuffer("RGBA", fig.canvas.get_width_height(), fig.canvas.buffer_rgba()).convert(
            "P", palette=Image.Palette.ADAPTIVE, colors=64))
        if t == t_star:
            fig.savefig(DOCS / f"{name}.png", dpi=110, bbox_inches="tight")
    imgs[0].save(DOCS / f"{name}.gif", save_all=True, append_images=imgs[1:], duration=120, loop=0, optimize=True)
    plt.close(fig)
    print(f"{out_json.name}: worst jam at t={t_star} with {len(who)} stuck robots in rows {r0}-{r1}, cols {c0}-{c1}")
    return DOCS / f"{name}.gif"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    r = sub.add_parser("run"); r.add_argument("names", nargs="*"); r.add_argument("--all", action="store_true")
    r.add_argument("--force", action="store_true"); r.add_argument("--no-gif", action="store_true")
    r.add_argument("--workers", type=int, default=6)
    d = sub.add_parser("render"); d.add_argument("names", nargs="*"); d.add_argument("--all", action="store_true")
    d.add_argument("--no-gif", action="store_true")
    hu = sub.add_parser("hunt"); hu.add_argument("outs", nargs="+", type=Path)
    a = ap.parse_args()
    if a.cmd == "list":
        for n, f in SCENES.items():
            sc = f()
            print(f"{n:16s} [{sc.kind}] {sc.title}  ({', '.join(r.key() for r in sc.runs)})")
        return
    if a.cmd == "hunt":
        for o in a.outs:
            hunt(o)
        return
    names = list(SCENES) if a.all or not a.names else a.names
    if a.cmd == "run":
        run_all(names, a.force, workers=a.workers)
    for n in names:
        res = render(SCENES[n](), gif=not a.no_gif)
        for k, v in res.items():
            print(f"{n:16s} {k:55s} {v}")


if __name__ == "__main__":
    main()
