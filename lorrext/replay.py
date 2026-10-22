"""Reconstruct a run tick by tick from a full output JSON (outputScreen 1),
validate it, and render side-by-side comparison videos.

    python -m lorrext.replay validate OUT.json
    python -m lorrext.replay video A.json B.json -o cmp.mp4 [--ticks 900] [--deadline 300]

The 2026 kit writes the *first* snapshot into every path segment header
(Simulator.cpp uses chunked_actual_snapshot_states[i][0] for all segments), so
positions are rebuilt by replaying actions from the start state only.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ORI = {"E": 0, "S": 1, "W": 2, "N": 3}
DR = [0, 1, 0, -1]
DC = [1, 0, -1, 0]


@dataclass
class Replay:
    rows: int
    cols: int
    obstacles: np.ndarray        # (rows, cols) bool
    pos: np.ndarray              # (T+1, A, 2) row, col
    T: int
    tasks: dict                  # id -> dict(reveal, done, locs[(r,c)], agent)
    events: list                 # (t, agent, task, seq)
    assign: np.ndarray           # (T+1, A) task id or -1
    label: str = ""
    finished: int = 0
    ori: np.ndarray | None = None  # (T+1, A) heading 0=E 1=S 2=W 3=N


def parse_actions(path, T: int) -> list[str]:
    """Per-tick action letters for one agent (both kit formats)."""
    acts: list[str] = []
    if isinstance(path, str) and path.startswith("["):  # 2026 segments
        for seg in re.findall(r"\[\([^)]*\):\(([^)]*)\)\]", path):
            for tok in seg.split(","):
                tok = tok.strip()
                if not tok:
                    continue
                a, n = tok.split()
                acts += [a] * int(n)
    else:
        acts = [a.strip() for a in str(path).split(",") if a.strip()]
    acts = acts[:T]
    return acts + ["W"] * (T - len(acts))


def read_map(path: Path) -> np.ndarray:
    lines = path.read_text().splitlines()
    h = int(lines[1].split()[1]); w = int(lines[2].split()[1])
    grid = lines[4:4 + h]
    return np.array([[ch not in ".GSE" for ch in row[:w]] for row in grid])


def load(out_json: Path, map_file: Path | None = None, label: str = "") -> Replay:
    js = json.loads(Path(out_json).read_text())
    T = js["lorrExt"]["simulationTime"] if "lorrExt" in js else js["makespan"]
    if map_file is None:
        map_file = Path(__file__).resolve().parent.parent / "instances" / "base" / "maps" / "random-32-32-20.map"
    obst = read_map(map_file)
    rows, cols = obst.shape
    A = js["teamSize"]
    pos = np.zeros((T + 1, A, 2), dtype=np.int16)
    ori = np.zeros((T + 1, A), dtype=np.int8)
    for a in range(A):
        r, c, o = js["start"][a]
        o = ORI[o] if isinstance(o, str) else int(o)
        pos[0, a] = (r, c)
        ori[0, a] = o
        for t, act in enumerate(parse_actions(js["actualPaths"][a], T)):
            if act == "F":
                r, c = r + DR[o], c + DC[o]
            elif act == "R":
                o = (o + 1) % 4
            elif act == "C":
                o = (o + 3) % 4
            pos[t + 1, a] = (r, c)
            ori[t + 1, a] = o
    tasks = {}
    for tid, rev, locs in js["tasks"]:
        tasks[tid] = dict(reveal=rev, done=-1, locs=list(zip(locs[0::2], locs[1::2])), agent=-1)
    for t_id, rec in ((r[0], r) for r in js["lorrExt"]["tasks"]):
        if t_id in tasks:
            tasks[t_id]["done"] = rec[2]
    events = [tuple(e) for e in js["events"]]
    assign = np.full((T + 1, A), -1, dtype=np.int32)
    first_event = {}  # (agent, task) -> first event tick
    for e in events:
        first_event.setdefault((e[1], e[2]), e[0])
    for a, sched in enumerate(js.get("actualSchedule", [])):
        cur_t, cur = 0, -1
        for item in str(sched).split(","):
            if ":" not in item:
                continue
            t, task = (int(x) for x in item.split(":"))
            if not 0 <= t <= T:  # the 2026 kit leaves the first entry's timestep uninitialised
                t = first_event.get((a, task), 0)
            t = min(max(t, cur_t), T)
            assign[cur_t:t, a] = cur
            cur_t, cur = t, task
        assign[cur_t:, a] = cur
    # a robot keeps a finished task id in the schedule until re-assigned; clear it
    for tid, tk in tasks.items():
        if tk["done"] >= 0:
            assign[tk["done"]:][assign[tk["done"]:] == tid] = -1
    return Replay(rows, cols, obst, pos, T, tasks, events, assign, label, js["numTaskFinished"], ori)


def validate(rp: Replay) -> dict:
    """Checks: no vertex conflicts, no obstacle cells, events on the robot's cell."""
    T, A = rp.pos.shape[0] - 1, rp.pos.shape[1]
    flat = rp.pos[..., 0].astype(np.int64) * rp.cols + rp.pos[..., 1]
    vertex = sum(int(A - len(np.unique(flat[t]))) for t in range(T + 1))
    inb = (rp.pos[..., 0] >= 0) & (rp.pos[..., 0] < rp.rows) & (rp.pos[..., 1] >= 0) & (rp.pos[..., 1] < rp.cols)
    obst = int((~inb).sum()) + int(rp.obstacles[np.clip(rp.pos[..., 0], 0, rp.rows - 1), np.clip(rp.pos[..., 1], 0, rp.cols - 1)][inb].sum())
    ok_ev = bad_ev = 0
    for t, a, tid, seq in rp.events:
        if t > T or tid not in rp.tasks:
            continue
        loc = rp.tasks[tid]["locs"][seq - 1]
        if tuple(rp.pos[t, a]) == tuple(loc):
            ok_ev += 1
        else:
            bad_ev += 1
    return {"ticks": T, "vertex_conflicts": vertex, "off_map_or_obstacle": obst,
            "events_on_cell": ok_ev, "events_off_cell": bad_ev}


# ------------------------------------------------------------------ video
def robot_states(rp: Replay, t: int) -> np.ndarray:
    """0 idle, 1 travelling, 2 at its current errand (serving / dwelling)."""
    st = np.zeros(rp.pos.shape[1], dtype=np.int8)
    for a, tid in enumerate(rp.assign[t]):
        if tid < 0 or tid not in rp.tasks:
            continue
        tk = rp.tasks[tid]
        k = sum(1 for (et, ea, eid, _) in tk.get("_ev", []) if et <= t)
        st[a] = 1
        if k < len(tk["locs"]) and tuple(rp.pos[t, a]) == tuple(tk["locs"][k]):
            st[a] = 2
    return st


def _index_events(rp: Replay) -> None:
    for tk in rp.tasks.values():
        tk["_ev"] = []
    for e in rp.events:
        if e[2] in rp.tasks:
            rp.tasks[e[2]]["_ev"].append(e)


def video(a: Replay, b: Replay, out: Path, ticks: int = 900, deadline: int = 300, fps: int = 30) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap

    for rp in (a, b):
        _index_events(rp)
    ticks = min(ticks, a.T, b.T)
    age_cmap = LinearSegmentedColormap.from_list("age", ["#f2d024", "#e8590c", "#c92a2a"])
    robot_colors = np.array(["#9aa0a6", "#1c7ed6", "#f08c00"])
    fig = plt.figure(figsize=(12.8, 7.2), dpi=100)
    gs = fig.add_gridspec(2, 2, height_ratios=[4, 1.1], hspace=0.25, wspace=0.08, left=0.04, right=0.98, top=0.87, bottom=0.07)
    axes = [fig.add_subplot(gs[0, i]) for i in range(2)]
    strip = fig.add_subplot(gs[1, :])
    arts = []
    oldest = {0: np.full(ticks + 1, np.nan), 1: np.full(ticks + 1, np.nan)}
    for i, rp in enumerate((a, b)):
        ax = axes[i]
        ax.imshow(rp.obstacles, cmap="Greys", vmin=0, vmax=1.6, interpolation="nearest")
        ax.set_xticks([]); ax.set_yticks([])
        tsc = ax.scatter([], [], marker="s", s=38, linewidths=0)
        late = ax.scatter([], [], marker="s", s=60, facecolors="none", edgecolors="k", linewidths=1.2)
        rsc = ax.scatter([], [], marker="o", s=30, edgecolors="white", linewidths=0.4, zorder=3)
        title = ax.set_title(rp.label, fontsize=11)
        arts.append((tsc, late, rsc, title))
        for t in range(ticks + 1):
            ages = [t - tk["reveal"] for tk in rp.tasks.values() if tk["reveal"] <= t and (tk["done"] < 0 or tk["done"] > t)]
            oldest[i][t] = max(ages) if ages else 0
    lines = [strip.plot([], [], color=c, lw=1.4, label=rp.label)[0] for c, rp in zip(["#1971c2", "#e03131"], (a, b))]
    strip.axhline(deadline, color="k", ls=":", lw=0.9)
    strip.set_xlim(0, ticks); strip.set_ylim(0, max(np.nanmax(oldest[0]), np.nanmax(oldest[1]), deadline) * 1.1)
    strip.set_ylabel("oldest open\ntask age"); strip.set_xlabel("tick"); strip.legend(fontsize=8, loc="upper left")
    cursor = strip.axvline(0, color="grey", lw=0.8)
    fig.text(0.5, 0.975, "robots: grey idle, blue travelling, orange at errand;  tasks: yellow -> red with age, black outline = past deadline",
             ha="center", fontsize=9)
    w, h = fig.canvas.get_width_height()
    out.parent.mkdir(parents=True, exist_ok=True)
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{w}x{h}",
                           "-r", str(fps), "-i", "-", "-pix_fmt", "yuv420p", "-vcodec", "libx264", "-crf", "23", str(out)],
                          stdin=subprocess.PIPE)
    for t in range(ticks + 1):
        for i, rp in enumerate((a, b)):
            tsc, late, rsc, title = arts[i]
            open_t = [(tk, t - tk["reveal"]) for tk in rp.tasks.values()
                      if tk["reveal"] <= t and (tk["done"] < 0 or tk["done"] > t)]
            pts = []
            cols = []
            lpts = []
            for tk, age in open_t:
                k = sum(1 for e in tk["_ev"] if e[0] <= t)
                r, c = tk["locs"][min(k, len(tk["locs"]) - 1)]
                pts.append((c, r)); cols.append(age_cmap(min(age / deadline, 1.0)))
                if age > deadline:
                    lpts.append((c, r))
            tsc.set_offsets(np.array(pts).reshape(-1, 2)); tsc.set_facecolors(cols if cols else "none")
            late.set_offsets(np.array(lpts).reshape(-1, 2))
            rsc.set_offsets(rp.pos[t][:, ::-1])
            rsc.set_facecolors(robot_colors[robot_states(rp, t)])
            done = sum(1 for tk in rp.tasks.values() if 0 <= tk["done"] <= t)
            title.set_text(f"{rp.label}\ntick {t}   done {done}   open {len(open_t)}   late {len(lpts)}")
            lines[i].set_data(np.arange(t + 1), oldest[i][: t + 1])
        cursor.set_xdata([t, t])
        fig.canvas.draw()
        ff.stdin.write(fig.canvas.buffer_rgba().tobytes())
    ff.stdin.close()
    ff.wait()
    plt.close(fig)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("validate"); v.add_argument("out", type=Path)
    m = sub.add_parser("video")
    m.add_argument("a", type=Path); m.add_argument("b", type=Path)
    m.add_argument("-o", type=Path, required=True)
    m.add_argument("--labels", nargs=2, default=None)
    m.add_argument("--ticks", type=int, default=900)
    m.add_argument("--deadline", type=int, default=300)
    a = ap.parse_args()
    if a.cmd == "validate":
        print(validate(load(a.out)))
    else:
        la, lb = a.labels or (a.a.stem, a.b.stem)
        print(video(load(a.a, label=la), load(a.b, label=lb), a.o, a.ticks, a.deadline))


if __name__ == "__main__":
    main()
