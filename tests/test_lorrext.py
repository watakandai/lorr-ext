import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from lorrext import arrivals
from lorrext.core import KITS, ROOT, Setting, deadline_metrics, metrics, run, service_intervals
from lorrext.replay import load, parse_actions, validate

TP = ROOT / "third_party"


# ------------------------------------------------------------------ settings
def test_service_intervals():
    assert service_intervals("none") == []
    assert service_intervals("kitchen") == [[2, 4], [10, 30]]
    assert service_intervals("3-5;10-20") == [[3, 5], [10, 20]]


def test_setting_key_only_shows_non_defaults():
    assert Setting(arrival="mmpp", rate=0.3).key() == "mmpp_r0.3_none_d0_s0_T1000"
    assert Setting(arrival="mmpp", rate=0.3, burst_factor=6).key().endswith("_burst_factor6")
    assert Setting(arrival="pool").key().startswith("pool1.5_")
    assert ";" not in Setting(service="2-4;10-30").key()


# ------------------------------------------------------------------ arrivals
@pytest.mark.parametrize("model", ["poisson", "mmpp", "hawkes"])
def test_arrival_mean_rate(model):
    x = arrivals.sample(Setting(arrival=model, rate=0.4), T=200_000, seed=1)
    assert abs(x.mean() - 0.4) < 0.03


def test_bursty_models_are_overdispersed():
    disp = {m: arrivals.burst_stats(arrivals.sample(Setting(arrival=m, rate=0.4), T=50_000, seed=2))["index_of_dispersion"]
            for m in ("poisson", "mmpp", "hawkes")}
    assert disp["poisson"] < 1.3 < disp["mmpp"] and disp["hawkes"] > 1.3


# ------------------------------------------------------------------ metrics
def fake_output(records, T=1000, finished=None):
    return {"numTaskFinished": finished if finished is not None else sum(r[2] >= 0 for r in records),
            "lorrExt": {"simulationTime": T, "tasks": records}}


def test_open_tasks_count_as_misses():
    # 10 tasks revealed at 200: 9 done after 50 ticks, one never finishes
    recs = [[i, 200, 250, 0, 2] for i in range(9)] + [[9, 200, -1, -1, 2]]
    m = metrics(fake_output(recs), 300)
    assert m["miss_300"] == pytest.approx(0.1)
    assert np.isinf(m["p99_300"])
    assert m["open_at_end"] == 1


def test_deadline_window_excludes_young_tasks():
    # a task revealed at 900 cannot have used a 300-tick deadline by T=1000
    tt = np.array([[100, 150, 2], [900, -1, 2]], dtype=float)
    d = deadline_metrics(tt, 1000)
    assert d["miss_300"] == 0.0


# ------------------------------------------------------------------ replay
def test_parse_actions_both_formats():
    seg = "[(0,1,2,0,0):(W 3,F 1)][(4,1,2,0,0):(R 2,F 1)]"
    assert parse_actions(seg, 8) == ["W", "W", "W", "F", "R", "R", "F", "W"]
    assert parse_actions("F,R,W", 4) == ["F", "R", "W", "W"]


# ------------------------------------------------------------------ patching
@pytest.mark.skipif(not (TP / "Start-Kit").exists(), reason="run scripts/setup.sh first")
def test_patch_is_idempotent_and_complete(tmp_path):
    from tools.patch_kit import MARK, patch

    kit = tmp_path / "kit"
    shutil.copytree(TP / "Start-Kit", kit, ignore=shutil.ignore_patterns("build", ".git", "example_problems"))
    patch(kit)
    first = {p: p.read_text() for p in kit.rglob("*.cpp")}
    patch(kit)
    assert first == {p: p.read_text() for p in kit.rglob("*.cpp")}
    tm = (kit / "src" / "TaskManager.cpp").read_text()
    assert MARK in tm and "LorrExt::reveal" in tm and "LorrExt::stop_reached" in tm
    assert "LorrExtSched::plan" in (kit / "src" / "TaskScheduler.cpp").read_text()


# ------------------------------------------------------------------ end to end
needs_kit = pytest.mark.skipif(not (KITS / "lorr2026" / "build" / "lifelong").exists(), reason="run scripts/setup.sh first")


@needs_kit
def test_simulation_end_to_end(tmp_path):
    s = Setting(arrival="poisson", rate=0.4, service="kitchen", steps=300, seed=3)
    r = run("fifo2026", s, tmp_path, screen=1)
    assert r["returncode"] == 0
    # arrivals follow the configured rate
    assert abs(r["revealed"] / 300 - 0.4) < 0.12
    rp = load(Path(r["output"]))
    v = validate(rp)
    assert v["vertex_conflicts"] == 0 and v["off_map_or_obstacle"] == 0 and v["events_off_cell"] == 0
    # every errand is served for at least the lower end of its interval
    iv, prev = [(2, 4), (10, 30)], {}
    for t, a, tid, seq in sorted(rp.events):
        loc = rp.tasks[tid]["locs"][seq - 1]
        start = prev.get(tid, rp.tasks[tid]["reveal"])
        on = sum(tuple(rp.pos[u, a]) == tuple(loc) for u in range(start + 1, t + 1))
        assert on >= iv[(seq - 1) % 2][0]
        prev[tid] = t


@needs_kit
def test_same_seed_same_arrivals_across_solvers(tmp_path):
    s = Setting(arrival="mmpp", rate=0.3, steps=200, seed=5)
    run("default2026", s, tmp_path)
    run("fifo2026", s, tmp_path)
    a = np.load(tmp_path / "tasks" / f"default2026__{s.key()}.npy")
    b = np.load(tmp_path / "tasks" / f"fifo2026__{s.key()}.npy")
    n = min(len(a), len(b))
    assert n > 20 and (a[:n, 1] == b[:n, 1]).all()
