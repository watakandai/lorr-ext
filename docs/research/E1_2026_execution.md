# E1: why the 2026 default finishes about half as many tasks as the 2024 default

**Question.** The 2024 and 2026 defaults run the same scheduler, the same PIBT and the same traffic-flow guidance (diffing the two trees shows only cosmetic changes; [algorithms.md §3](algorithms.md#3-the-2026-default-and-the-two-baseline-schedulers-on-its-planner)). Yet on the competition instance the 2026 kit gets 0.44 tasks/tick and the 2024 kit 0.81. What costs the other half?

**Hypothesis.** The 2026 execution model. The planner works on a background thread for one *window* (10 ticks with the competition timing); its plan reaches the robots one window later, and inside a window goals are frozen, so a robot that finishes an errand waits for the next plan to learn where to go. Every errand transition then costs up to two windows of standing still. If that latency is the whole gap, a 1-tick window should close it.

**Experiment.** `python -m lorrext.investigate e1`: the competition setting (random-32-32-20, 100 robots, pool), 2026 default, varying the tick length and the planning window. One subtlety: the planner reserves at least 100 ms for PIBT and gives traffic guidance the rest of the window ([`planner.cpp#L362`](https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/default_planner/planner.cpp#L362)), so short windows also starve guidance. Slowing the clock (300 ms ticks) keeps a 1-tick window long enough for guidance.

| Tick | Window | Window in ticks | Guidance time per plan | Tasks/tick | p50 latency |
|---|---|---|---|---|---|
| 30 ms | 300 ms | 10 | ≈190 ms | 0.439 | 189 |
| 100 ms | 1000 ms (competition) | 10 | ≈890 ms | 0.444 | 190 |
| 300 ms | 900 ms | 3 | ≈790 ms | 0.554 | 157 |
| 300 ms | 300 ms | 1 | ≈190 ms | **0.651** | 138 |
| 100 ms | 100 ms | 1 | **0** | 0.276 | 204 |
| 2024 default (same algorithm, every tick) | | 1 | ≈90 ms | **0.805** | 112 |

## Verdict

1. **Our scaled timing is faithful.** 30/300 ms and the competition's 100/1000 ms give the same result (0.439 vs 0.444): what matters is the window length *in ticks*, not in milliseconds.
2. **Plan latency explains about 58% of the gap.** Shrinking the window from 10 ticks to 1, with guidance kept, raises throughput from 0.44 to 0.65; the 2024 default is at 0.81.
3. **Traffic guidance is not optional.** At a 1-tick window *without* guidance time, throughput collapses to 0.28, below even the 10-tick run. The 2026 kit's budget rule (PIBT first, guidance gets the rest) silently disables guidance whenever the window is short.
4. **The remaining 0.65 → 0.81 is the execution layer itself.** With 1-tick plans, the differences left are the executor: it drops planned waits, re-orders execution by the temporal plan graph, and rolls PIBT out internally with turns appended as delays. Not isolated further here.

## What it means

The 2026 kit's design is right for imperfect execution (see B1/B2: it is the only one that survives breakdowns), but the default planner pays for it on every goal change. Two fixes follow (see [`proposals.md`](proposals.md)):
- **Plan the next errand ahead.** The rollout could switch a robot to its next errand when it reaches the current one (the task's errands are known), instead of freezing goals for the window.
- **Budget guidance explicitly.** Reserve guidance time as a fraction of the window, not "whatever PIBT's 100 ms floor leaves".
