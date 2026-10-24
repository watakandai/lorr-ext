# Results of the research grid (stages 2–3)

540 runs, `python -m lorrext.research`, mean of 2 seeds, tasks/tick unless noted. Default setting: random-32-32-20, 100 robots, task pool. Kitty = Kitty_Knight, NMS = No Man's Sky. Tables: `python -m lorrext.research_report <part>`.

## Main findings

| Question | Result | Weakness | Root cause / status |
|---|---|---|---|
| C2 layouts | Ranking holds: Kitty ≈ NMS > 2024 default > 2026 solvers on all four layouts. Renaming the map costs NMS 4–6% (1.07→1.03; 2.01→1.88 at 200 robots) | NMS's tuning is keyed on the map name | Confirmed: its lane map and parameters are selected by file name |
| C3 fleet size | NMS keeps scaling to 300 robots (2.54) and is first from 150 on; Kitty peaks at 300 (1.90); the 2026 solvers peak at 150 and then fall, least-slack to 0.12 at 400 | Congestion collapse in the 2026 kit | E1: the 10-tick plan latency, plus guidance starved by the fixed 100 ms PIBT reserve |
| B1 breakdowns | Kitty crashes on every fault type. At a failure rate of 0.002 per tick, NMS loses 67% and the 2026 solvers 73–85%. The 2024 all-stop rule is catastrophic: 0.03 at a rate of 0.001 | No planner models frozen robots; Kitty exits on a plan mismatch | Confirmed ([B1](B1_breakdowns.md)): planners route robots into frozen ones (cascade waits 2.7–3.7× the frozen time); a fault-aware planner removes them. Kitty: resync fix, 0.61 instead of a crash |
| B2 slow robots | Loss grows with the slow share; NMS −40% at 50% slow | Plans assume equal speeds | Same mechanism as B1, milder |
| B3 pauses | 10% downtime per robot costs 25–30% | Downtime spreads to other robots | Robots queue behind a paused one (cascade waits) |
| C1 aisle closure | −28 to −33% for a 400-tick closure | Heuristics stay on the static map | Confirmed: robots wait at the closure's edge |
| A1 deadlines | Urgent tasks are missed 38–94% of the time; standard tasks 0–37%. Least-slack does **not** beat oldest-first (urgent misses 0.85–0.94 vs 0.82–0.99) | A scheduler that reads deadlines can't use them when plans arrive 10 ticks late | 2026 kit: the planner is the bottleneck, not the order of tasks. NMS misses the fewest because it is fastest |
| A2 stations | Slow stations (20 ticks) cost 18–26% under a pool; under Poisson demand all solvers keep up | Station queues | Kitty runs out of memory (−9) or stalls (0.0) on warehouse Poisson runs: **open** |
| A3 tours | 5-stop tours cut throughput by half for every solver; the 2026 kit drops the most (0.44→0.12) | A long opened task is locked to its robot | Expected: 5 errands per task |
| A4 long trips | Ranking unchanged; under Poisson demand all solvers keep up | — | Empty driving is not the bottleneck at these rates |

## Fixes (stage 5)

Measured on the settings above by `python -m lorrext.research_fixes`; details in [proposals.md](proposals.md).

| Fix | Effect | Doc |
|---|---|---|
| Fault-aware 2024 default (`default2024fa`) | Breakdowns +14% to +82% (0.27 → 0.49 at 0.002, above No Man's Sky's 0.35); all-stop rule 0.03 → 0.59; slow robots −32/−35% | [B1](B1_breakdowns.md) |
| NMS first-arrival scoring (`nms2024fix`) | Livelock gone; +2–6% on every layout | [E2](E2_nms_livelock.md) |
| 2026: 2-tick window + 20 ms PIBT reserve (`default2026g`) | 0.44 → 0.59; breakdowns +68–78%; −16% at 300–400 robots | [E1](E1_2026_execution.md) |
| 2026 plan-ahead (`default2026nx`) | 5-stop tours +26% | [E1](E1_2026_execution.md) |
| Kitty resync (`kitty2024rs`) | Survives breakdowns (0.61 at a failure rate of 0.001) | [B1](B1_breakdowns.md) |
