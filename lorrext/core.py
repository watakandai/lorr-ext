"""Instances, solver registry, running one simulation, and per-run metrics."""

from __future__ import annotations

import json
import os
import subprocess
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from . import cellmaps

ROOT = Path(__file__).resolve().parent.parent
KITS = ROOT / "kits"
BASE = ROOT / "instances" / "base"
MAP_FILE = BASE / "maps" / "random-32-32-20.map"
FREE = cellmaps.read_free(MAP_FILE) if MAP_FILE.exists() else np.ones((32, 32), bool)

# ----------------------------------------------------------------- solvers
# gen "2026": real-time kit (ticks of TICK_MS, planner window WINDOW_MS)
# gen "2024": planner-paced kit (PLAN_MS per tick)
SOLVERS = {
    "default2026": dict(kit="lorr2026", gen="2026", scheduler="default", label="2026 default (greedy)"),
    "fifo2026": dict(kit="lorr2026", gen="2026", scheduler="fifo", label="oldest-first"),
    "slack2026": dict(kit="lorr2026", gen="2026", scheduler="slack", label="least-slack"),
    "default2024": dict(kit="default2024", gen="2024", scheduler="default", label="2024 default"),
    "nms2024": dict(kit="nms2024", gen="2024", scheduler="default", label="No Man's Sky (2024 1st)"),
    "kitty2024": dict(kit="kitty2024", gen="2024", scheduler="default", label="Kitty_Knight (2024 2nd, guarded)"),
}

TICK_MS = 30      # 2026 kit: wall-clock per tick
WINDOW_MS = 300   # 2026 kit: planner communication window
PLAN_MS = 100     # 2024 kit: planner time per tick


@dataclass
class Setting:
    arrival: str = "pool"           # pool (competition default) | poisson | mmpp | hawkes
    rate: float = 0.5               # mean tasks per tick (ignored for pool)
    service: str = "none"           # per-cell service-time map, see lorrext/cellmaps.py
    delay: str = "none"             # per-cell move-delay map (2026 kit only); none = deterministic
    p_delay: float = 0.0            # the 2026 kit's own location-blind delays (kept at 0)
    seed: int = 0
    deadline: int = 300
    steps: int = 1000
    pool_reveal: float = 1.5        # numTasksReveal for pool mode
    burst_factor: float = 4.0       # mmpp: burst rate / mean rate
    burst_fraction: float = 0.2     # mmpp: share of time in burst
    burst_length: float = 50.0      # mmpp: mean burst length (ticks)
    branching: float = 0.6          # hawkes: offspring per task
    decay: float = 0.05             # hawkes: kernel decay per tick

    def key(self) -> str:
        svc = Path(self.service).stem if self.service.endswith(".json") else self.service.replace(":", "_")
        s = f"{self.arrival}_r{self.rate:g}_{svc}_d{self.p_delay:g}_s{self.seed}_T{self.steps}"
        if self.arrival == "pool":
            s = f"pool{self.pool_reveal:g}_{svc}_d{self.p_delay:g}_s{self.seed}_T{self.steps}"
        if self.delay != "none":
            s += "_delay-" + (Path(self.delay).stem if self.delay.endswith(".json") else self.delay.replace(":", "_"))
        d = Setting()
        extra = {"mmpp": ("burst_factor", "burst_fraction", "burst_length"), "hawkes": ("branching", "decay")}
        for f in extra.get(self.arrival, ()):
            if getattr(self, f) != getattr(d, f):
                s += f"_{f}{getattr(self, f):g}"
        if self.deadline != d.deadline:
            s += f"_D{self.deadline}"
        return s


def arrival_config(s: Setting) -> dict:
    a = {"model": s.arrival, "rate": s.rate, "seed": s.seed}
    if s.arrival == "mmpp":
        a.update(burstFactor=s.burst_factor, burstFraction=s.burst_fraction, burstLength=s.burst_length)
    if s.arrival == "hawkes":
        a.update(branching=s.branching, decay=s.decay)
    return a


def write_instance(s: Setting, scheduler: str, out_dir: Path) -> Path:
    """Instance JSON next to symlinks of the base map/agents/tasks."""
    out_dir.mkdir(parents=True, exist_ok=True)
    for sub in ("maps", "agents", "tasks"):
        link = out_dir / sub
        try:
            link.symlink_to(BASE / sub)
        except FileExistsError:
            pass
    arrival = arrival_config(s)
    inst = {
        "mapFile": "maps/random-32-32-20.map",
        "agentFile": "agents/random_32_32_20_100.agents",
        "teamSize": 100,
        "taskFile": "tasks/random_32_32_20_official.tasks",
        "numTasksReveal": s.pool_reveal,
        "agentCounter": 1,
        "agentSize": 1.0,
        "delayConfig": {"seed": s.seed, "minDelay": 1, "maxDelay": 4, "eventModel": "bernoulli",
                        "pDelay": s.p_delay, "durationModel": "uniform",
                        "gaussMeanRatio": 0.5, "gaussStdRatio": 0.22},
        "lorrExt": {"arrival": arrival,
                    "mapCols": FREE.shape[1],
                    "service": {**(cellmaps.build(s.service, "service", FREE, s.seed) or {}), "seed": s.seed},
                    "delay": {**(cellmaps.build(s.delay, "delay", FREE, s.seed) or {}), "seed": s.seed},
                    "deadline": s.deadline, "scheduler": scheduler},
    }
    p = out_dir / f"{s.key()}_{scheduler}.json"
    p.write_text(json.dumps(inst, indent=1))
    return p


def run(solver: str, s: Setting, out_dir: Path, screen: int = 3, timeout: int = 1800, cpus: str | None = None) -> dict:
    """Run one simulation; returns the metrics row (also cached as JSON)."""
    sv = SOLVERS[solver]
    if sv["gen"] == "2024" and (s.delay != "none" or s.p_delay > 0):
        raise ValueError(f"{solver}: the 2024 kit has no move-delay model; use a 2026-kit solver for delays")
    out_dir = Path(out_dir).resolve()
    tag = f"{solver}__{s.key()}"
    row_path = out_dir / "rows" / f"{tag}.json"
    if row_path.exists() and screen == 3:
        return json.loads(row_path.read_text())
    (out_dir / "rows").mkdir(parents=True, exist_ok=True)
    (out_dir / "out").mkdir(parents=True, exist_ok=True)
    inst = write_instance(s, sv["scheduler"], out_dir / "instances")
    out = out_dir / "out" / f"{tag}.json"
    kit = KITS / sv["kit"]
    cmd = [str(kit / "build" / "lifelong"), "-i", str(inst), "-o", str(out), "-s", str(s.steps),
           "-c", str(screen), "-d", "3", "-p", "60000"]
    if sv["gen"] == "2026":
        cmd += ["-a", str(TICK_MS), "-t", str(WINDOW_MS), "-n", str(WINDOW_MS), "-x", str(TICK_MS)]
        if screen <= 2:
            cmd += ["-w", "100000"]  # keep the whole action log
    else:
        cmd += ["-t", str(PLAN_MS)]
    if cpus:  # pin to dedicated cores (No Man's Sky alone spawns ~10 threads)
        cmd = ["taskset", "-c", cpus] + cmd
    t0 = time.time()
    try:
        cp = subprocess.run(cmd, cwd=kit, capture_output=True, text=True, timeout=timeout,
                            env={**os.environ, "OMP_NUM_THREADS": "1"})
        rc, err = cp.returncode, cp.stdout[-2000:] + cp.stderr[-2000:]
    except subprocess.TimeoutExpired:
        rc, err = -9, "timeout"
    row = {"solver": solver, **asdict(s), "cpus": cpus, "returncode": rc, "wall_s": round(time.time() - t0, 1), "output": str(out)}
    if out.exists():
        js = json.loads(out.read_text())
        row.update(metrics(js, s.deadline, cellmaps.per_cell(cellmaps.build(s.service, "service", FREE, s.seed), FREE)))
        # per-task records (id, reveal, done, agent, stops) for task-level diffs
        (out_dir / "tasks").mkdir(exist_ok=True)
        np.save(out_dir / "tasks" / f"{tag}.npy", np.array(js["lorrExt"]["tasks"], dtype=np.int32).reshape(-1, 5))
    else:
        row["error"] = err
    if screen == 3:
        out.unlink(missing_ok=True)  # summary rows are enough
        row["output"] = None
    row_path.write_text(json.dumps(row))
    return row


# ----------------------------------------------------------------- task geometry
_GEOM: dict = {}


def _geometry():
    """Map free cells, BFS distance cache and task errand lists (from the base instance)."""
    if not _GEOM:
        lines = (BASE / "maps" / "random-32-32-20.map").read_text().splitlines()
        h, w = int(lines[1].split()[1]), int(lines[2].split()[1])
        free = [[ch in ".GSE" for ch in row[:w]] for row in lines[4:4 + h]]
        tlines = (BASE / "tasks" / "random_32_32_20_official.tasks").read_text().splitlines()[2:]
        _GEOM.update(h=h, w=w, free=free, dist={}, tasks=[[int(x) for x in l.split(",")] for l in tlines if l.strip()])
    return _GEOM


def bfs_from(src: int) -> dict:
    """Shortest number of moves (ignoring rotations) from cell src to every cell."""
    g = _geometry()
    if src in g["dist"]:
        return g["dist"][src]
    h, w, free = g["h"], g["w"], g["free"]
    d = {src: 0}
    frontier = [src]
    while frontier:
        nxt = []
        for c in frontier:
            r, k = divmod(c, w)
            for rr, kk in ((r + 1, k), (r - 1, k), (r, k + 1), (r, k - 1)):
                if 0 <= rr < h and 0 <= kk < w and free[rr][kk]:
                    n = rr * w + kk
                    if n not in d:
                        d[n] = d[c] + 1
                        nxt.append(n)
        frontier = nxt
    g["dist"][src] = d
    return d


def task_length(task_id: int) -> int:
    """Intrinsic travel of a task: moves from its first errand through the rest."""
    locs = _geometry()["tasks"][task_id % len(_geometry()["tasks"])]
    return sum(bfs_from(a).get(b, 0) for a, b in zip(locs, locs[1:]))


def expected_service(task_id: int, svc_grid: np.ndarray | None) -> float:
    """Expected total service time of a task from the per-cell mean grid."""
    if svc_grid is None:
        return 0.0
    w = svc_grid.shape[1]
    locs = _geometry()["tasks"][task_id % len(_geometry()["tasks"])]
    return float(sum(np.nan_to_num(svc_grid[v // w, v % w]) for v in locs))


# ----------------------------------------------------------------- metrics
def task_table(js: dict) -> np.ndarray:
    """(n, 3) array: t_revealed, t_completed (-1 open), n_stops."""
    recs = js["lorrExt"]["tasks"]
    return np.array([[r[1], r[2], r[4]] for r in recs], dtype=float).reshape(-1, 3)


def latencies(js: dict, warmup_frac: float = 0.1) -> tuple[np.ndarray, np.ndarray]:
    """Latency per task revealed after warm-up. Open tasks are right-censored at
    the horizon (latency >= T - reveal), returned with a censored flag."""
    T = js["lorrExt"]["simulationTime"]
    tt = task_table(js)
    tt = tt[tt[:, 0] >= warmup_frac * T]
    done = tt[:, 1] >= 0
    lat = np.where(done, tt[:, 1] - tt[:, 0], T - tt[:, 0])
    return lat, ~done


DEADLINES = (100, 150, 200, 250, 300, 400, 500)


def deadline_metrics(tt: np.ndarray, T: int) -> dict:
    """Per deadline D: miss rate and p99 over tasks revealed in [0.1T, T-D]
    (each had the full D ticks to finish); open tasks count as misses / inf."""
    out = {}
    for D in DEADLINES:
        e = tt[(tt[:, 0] >= 0.1 * T) & (tt[:, 0] <= T - D)]
        lat = np.where(e[:, 1] >= 0, e[:, 1] - e[:, 0], np.inf)
        out[f"miss_{D}"] = float((lat > D).mean()) if len(e) else float("nan")
        out[f"p99_{D}"] = float(np.percentile(lat, 99, method="higher")) if len(e) else float("nan")
    return out


def metrics(js: dict, deadline: int, svc_grid: np.ndarray | None = None) -> dict:
    T = js["lorrExt"]["simulationTime"]
    lat, open_ = latencies(js)
    tt = task_table(js)
    # deadline misses only over tasks old enough to have had the full deadline
    eligible = tt[(tt[:, 0] >= 0.1 * T) & (tt[:, 0] <= T - deadline)]
    miss = np.where(eligible[:, 1] >= 0, eligible[:, 1] - eligible[:, 0] > deadline, True)
    q = lambda p: float(np.percentile(lat, p)) if len(lat) else float("nan")
    # CVaR_95: mean latency of the worst 5% of tasks (open tasks at their age so far)
    worst = np.sort(lat)[-max(1, int(np.ceil(0.05 * len(lat)))):] if len(lat) else np.array([np.nan])
    # normalised latency: latency / (own errand-to-errand travel + expected service)
    recs = [r for r in js["lorrExt"]["tasks"] if r[1] >= 0.1 * T]
    ideal = np.array([max(1.0, task_length(r[0]) + expected_service(r[0], svc_grid)) for r in recs])
    stretch = lat / ideal if len(lat) == len(ideal) and len(lat) else np.array([np.nan])
    # time-averaged number of open tasks over the scored window
    ts = np.arange(int(0.1 * T), T)
    open_t = ((tt[:, 0][None, :] <= ts[:, None]) & ((tt[:, 1][None, :] < 0) | (tt[:, 1][None, :] > ts[:, None]))).sum(1) if len(tt) else np.zeros(1)
    return {
        "finished": int(js["numTaskFinished"]),
        "throughput": js["numTaskFinished"] / T,
        "revealed": int(len(tt)),
        "open_at_end": int((tt[:, 1] < 0).sum()),
        "lat_mean": float(lat.mean()) if len(lat) else float("nan"),
        "lat_p50": q(50), "lat_p90": q(90), "lat_p99": q(99), "lat_max": float(lat.max()) if len(lat) else float("nan"),
        "lat_cvar95": float(worst.mean()),
        "stretch_p50": float(np.median(stretch)), "stretch_p99": float(np.percentile(stretch, 99)),
        "open_mean": float(open_t.mean()),
        "censored_frac": float(open_.mean()) if len(lat) else float("nan"),
        "miss_rate": float(miss.mean()) if len(miss) else float("nan"),
        **deadline_metrics(tt, T),
        "planner_errors": js.get("numPlannerErrors"), "schedule_errors": js.get("numScheduleErrors"),
        "timeouts": js.get("numEntryTimeouts"),
    }
