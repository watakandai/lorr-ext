"""Instance families beyond the competition setting: map layouts, fleet sizes, task patterns.

Layouts (Setting.layout)
    random          the competition's random-32-32-20 map (instances/base); the default
    random_renamed  the same map saved under another file name. Entries that tune
                    themselves by map name (No Man's Sky's lanes and weights, Kitty_Knight's
                    learned weights) then run untuned, everything else is identical.
    warehouse       warehouse_small (33 x 57, 2024 problem generator): shelf cells 'S' in
                    blocks in the middle, picking stations 'E' along the edges
    sortation       sortation_small (33 x 57): induction points 'E' along the edges,
                    chutes 'S' in a grid in the middle

Task patterns (Setting.tasks)
    official        the competition task file (random layouts only)
    uniform:a-b     a..b errands at uniformly random free cells
    g2p             goods-to-person picking: shelf S -> station E -> back to the same shelf S
    sort            sortation: induction point E -> chute S
    tour:k          chain-store replenishment: one of 4 depots, then k shop stops
    trip:dmin       ride-hailing: a pickup and a drop-off at least dmin moves apart

Generated files live in instances/gen/<layout>/{maps,agents,tasks}/ and are reproducible
from (layout, agents, tasks, seed).
"""

from __future__ import annotations

import json
import shutil
from collections import deque
from functools import lru_cache
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
BASE = ROOT / "instances" / "base"
GEN = ROOT / "instances" / "gen"
GENERATOR = ROOT / "third_party" / "Benchmark-Archive" / "2024 Competition" / "Problem Generator" / "script"
KITTY = ROOT / "kits" / "kitty2024"

LAYOUTS = {
    "random": dict(src=BASE / "maps" / "random-32-32-20.map", name="random-32-32-20.map"),
    "random_renamed": dict(src=BASE / "maps" / "random-32-32-20.map", name="grid-32-32-20.map"),
    "warehouse": dict(src=GENERATOR / "warehouse_small.map", name="warehouse_small.map"),
    "sortation": dict(src=GENERATOR / "sortation_small.map", name="sortation_small.map"),
}
OFFICIAL_AGENTS = {100, 200, 400}  # random_32_32_20_<n>.agents from the 2024 main round
N_TASKS = 20000


def chars(layout: str) -> np.ndarray:
    lines = LAYOUTS[layout]["src"].read_text().splitlines()
    h, w = int(lines[1].split()[1]), int(lines[2].split()[1])
    return np.array([list(row[:w]) for row in lines[4:4 + h]])


def free(layout: str) -> np.ndarray:
    return ~np.isin(chars(layout), ["@", "T"])


@lru_cache(maxsize=None)
def _bfs(layout: str, src: int) -> np.ndarray:
    fr = free(layout)
    h, w = fr.shape
    d = np.full(h * w, -1, dtype=np.int32)
    d[src] = 0
    q = deque([src])
    while q:
        c = q.popleft()
        r, k = divmod(c, w)
        for rr, kk in ((r + 1, k), (r - 1, k), (r, k + 1), (r, k - 1)):
            if 0 <= rr < h and 0 <= kk < w and fr[rr, kk]:
                n = rr * w + kk
                if d[n] < 0:
                    d[n] = d[c] + 1
                    q.append(n)
    return d


def dist(layout: str, a: int, b: int) -> int:
    return int(_bfs(layout, a)[b])


def _cells(layout: str, which: str | None = None) -> np.ndarray:
    ch = chars(layout)
    w = ch.shape[1]
    mask = free(layout) if which is None else (ch == which)
    rc = np.argwhere(mask)
    return rc[:, 0] * w + rc[:, 1]


def _largest_component(layout: str) -> np.ndarray:
    """Free cells reachable from the most common component (so every errand is reachable)."""
    cells = _cells(layout)
    seen, best = set(), np.array([], dtype=int)
    for c in cells:
        if c in seen:
            continue
        reach = np.flatnonzero(_bfs(layout, int(c)) >= 0)
        seen.update(reach.tolist())
        if len(reach) > len(best):
            best = reach
    return best


def gen_tasks(layout: str, pattern: str, seed: int, n: int = N_TASKS) -> list[list[int]]:
    rng = np.random.default_rng(10_000 + seed)
    comp = _largest_component(layout)
    inside = lambda arr: arr[np.isin(arr, comp)]
    if pattern.startswith("uniform:"):
        lo, hi = (int(x) for x in pattern.split(":")[1].split("-"))
        return [rng.choice(comp, size=rng.integers(lo, hi + 1)).tolist() for _ in range(n)]
    if pattern == "g2p":
        shelves, stations = inside(_cells(layout, "S")), inside(_cells(layout, "E"))
        out = []
        for _ in range(n):
            s = int(rng.choice(shelves))
            out.append([s, int(rng.choice(stations)), s])
        return out
    if pattern == "sort":
        inducts, chutes = inside(_cells(layout, "E")), inside(_cells(layout, "S"))
        return [[int(rng.choice(inducts)), int(rng.choice(chutes))] for _ in range(n)]
    if pattern.startswith("tour:"):
        k = int(pattern.split(":")[1])
        depots = rng.choice(comp, size=4, replace=False)
        return [[int(rng.choice(depots))] + rng.choice(comp, size=k).tolist() for _ in range(n)]
    if pattern.startswith("trip:"):
        dmin = int(pattern.split(":")[1])
        out = []
        while len(out) < n:
            a = int(rng.choice(comp))
            far = np.flatnonzero(_bfs(layout, a) >= dmin)
            if len(far):
                out.append([a, int(rng.choice(far))])
        return out
    raise ValueError(f"unknown task pattern {pattern!r}")


def _write_map(layout: str, dest: Path) -> None:
    dest.parent.mkdir(parents=True, exist_ok=True)
    if not dest.exists():
        shutil.copy(LAYOUTS[layout]["src"], dest)


def _kitty_files(layout: str, name: str) -> None:
    """Kitty_Knight reads wppl_planner/configs/<map>.json and data/map_stats/<map>.json by map
    name. The competition map keeps its own config; every other layout gets the random-map
    config with the learned edge weights removed (they are tied to one map and fleet size)."""
    if not KITTY.exists():
        return
    stem = Path(name).stem
    cfg_dir, stats_dir = KITTY / "wppl_planner" / "configs", KITTY / "data" / "map_stats"
    if layout != "random":
        cfg = json.loads((cfg_dir / "random-32-32-20.json").read_text())
        cfg["map_weights_path"] = [{"n_agents": 1000000, "value": ""}]
        (cfg_dir / f"{stem}.json").write_text(json.dumps(cfg, indent=1))
    stats = stats_dir / f"{stem}.json"
    if not stats.exists():
        e, s = _cells(layout, "E").tolist(), _cells(layout, "S").tolist()
        stats.write_text(json.dumps({"num_empty_locs": int(free(layout).sum()), "num_E_locs": len(e),
                                     "num_S_locs": len(s), "E_locs": e, "S_locs": s}))


def prepare(layout: str, agents: int, tasks: str, seed: int) -> dict:
    """Make sure the map/agents/tasks files exist; return their directory and names."""
    if layout == "random" and tasks == "official" and agents in OFFICIAL_AGENTS:
        a = f"random_32_32_20_{agents}.agents"
        if not (BASE / "agents" / a).exists():
            raise FileNotFoundError(f"{BASE / 'agents' / a} missing: rerun scripts/setup.sh")
        _kitty_files(layout, LAYOUTS[layout]["name"])
        return dict(dir=BASE, mapFile="maps/random-32-32-20.map", agentFile=f"agents/{a}",
                    taskFile="tasks/random_32_32_20_official.tasks")
    if tasks == "official" and not layout.startswith("random"):
        raise ValueError("the official task file belongs to the random map")
    d = GEN / layout
    name = LAYOUTS[layout]["name"]
    _write_map(layout, d / "maps" / name)
    _kitty_files(layout, name)
    agent_file = d / "agents" / f"{layout}_{agents}_s{seed}.agents"
    if layout.startswith("random") and agents in OFFICIAL_AGENTS and seed == 0:
        agent_file.parent.mkdir(parents=True, exist_ok=True)
        if not agent_file.exists():
            shutil.copy(BASE / "agents" / f"random_32_32_20_{agents}.agents", agent_file)
    elif not agent_file.exists():
        rng = np.random.default_rng(20_000 + seed)
        starts = rng.choice(_largest_component(layout), size=agents, replace=False)
        agent_file.parent.mkdir(parents=True, exist_ok=True)
        agent_file.write_text(f"# version for LoRR 2024\n{agents}\n" + "\n".join(map(str, starts)) + "\n")
    tag = tasks.replace(":", "-")
    if tasks == "official":
        task_file = d / "tasks" / "random_32_32_20_official.tasks"
        task_file.parent.mkdir(parents=True, exist_ok=True)
        if not task_file.exists():
            shutil.copy(BASE / "tasks" / "random_32_32_20_official.tasks", task_file)
    else:
        task_file = d / "tasks" / f"{layout}_{tag}_s{seed}.tasks"
        if not task_file.exists():
            ts = gen_tasks(layout, tasks, seed)
            task_file.parent.mkdir(parents=True, exist_ok=True)
            task_file.write_text(f"# version for LoRR 2024\n{len(ts)}\n" + "\n".join(",".join(map(str, t)) for t in ts) + "\n")
    return dict(dir=d, mapFile=f"maps/{name}", agentFile=f"agents/{agent_file.name}", taskFile=f"tasks/{task_file.name}")


def task_list(layout: str, tasks: str, seed: int) -> list[list[int]]:
    """Errand cells of every task in the file a Setting uses (for task-length metrics)."""
    info = prepare(layout, 100 if layout.startswith("random") else 1, tasks, seed) if tasks == "official" else None
    path = (info["dir"] / info["taskFile"]) if info else GEN / layout / "tasks" / f"{layout}_{tasks.replace(':', '-')}_s{seed}.tasks"
    lines = path.read_text().splitlines()[2:]
    return [[int(x) for x in l.split(",")] for l in lines if l.strip()]


@lru_cache(maxsize=None)
def path_load(layout: str, samples: int = 600, seed: int = 0) -> np.ndarray:
    """How many of `samples` random shortest paths (between uniform free cells) pass each cell:
    a cheap betweenness estimate, i.e. where traffic concentrates."""
    rng = np.random.default_rng(seed)
    comp = _largest_component(layout)
    fr = free(layout)
    h, w = fr.shape
    load = np.zeros(h * w)
    for _ in range(samples):
        a, b = (int(x) for x in rng.choice(comp, size=2, replace=False))
        d = _bfs(layout, b)
        c = a
        while c != b:  # walk downhill in the distance-to-b field
            load[c] += 1
            r, k = divmod(c, w)
            nxt = [rr * w + kk for rr, kk in ((r + 1, k), (r - 1, k), (r, k + 1), (r, k - 1))
                   if 0 <= rr < h and 0 <= kk < w and fr[rr, kk] and d[rr * w + kk] == d[c] - 1]
            c = nxt[int(rng.integers(len(nxt)))]
        load[b] += 1
    return load.reshape(h, w)


def closure_cells(layout: str, name: str) -> list[list[int]]:
    """Named cell sets that can be closed (Setting.faults 'close<name>@a-b'):
        hot<k>  the k busiest cells (path_load) connected to the busiest one: a busy stretch
                of aisle closed for a spill or maintenance
    """
    if name.startswith("hot"):
        k = int(name[3:])
        load = path_load(layout)
        h, w = load.shape
        busy = load >= np.percentile(load[load > 0], 80)
        start = int(np.argmax(load))
        out, q, seen = [], deque([start]), {start}
        while q and len(out) < k:
            c = q.popleft()
            out.append(c)
            r, kk = divmod(c, w)
            nb = [rr * w + cc for rr, cc in ((r + 1, kk), (r - 1, kk), (r, kk + 1), (r, kk - 1))
                  if 0 <= rr < h and 0 <= cc < w and busy[rr, cc] and rr * w + cc not in seen]
            for n in sorted(nb, key=lambda n: -load.flat[n]):
                seen.add(n)
                q.append(n)
        return [[c // w, c % w] for c in out]
    raise ValueError(f"unknown closure set {name!r}")
