# Proposals (stage 5): fixes for the weaknesses found

Each fix is a patcher option (`tools/patch_kit.py`) built as its own solver by `scripts/setup.sh`, and measured by `python -m lorrext.research_fixes` on the grid settings that exposed the weakness (mean of 2 seeds, tasks/tick).

## Built and measured

| Weakness | Root cause | Fix (solver, flag) | Result |
|---|---|---|---|
| Breakdowns cost 10× their downtime; the 2024 all-stop rule stops the fleet ([B1](B1_breakdowns.md)) | The planner plans frozen robots as moving, and the robots behind them into their cells | Pin frozen robots and release moves committed into them (`default2024fa`, `--fault-aware`) | +14% to +82% under breakdowns; all-stop rule 0.03 → 0.59; pauses +12%. **Hurts slow robots (−32/−35%)**: apply only to stops longer than a tick |
| No Man's Sky livelocks on open maps ([E2](E2_nms_livelock.md)) | Its arrival rule scores the last arrival, later better | Score the first arrival, earlier better (`nms2024fix`, `--nms-arrival-fix`) | +5.0% on the competition map; +2–6% on all six layout/fleet settings; −9/−10% at middle breakdown rates (2 seeds, unexplained) |
| The 2026 default does about half the 2024 default's work ([E1](E1_2026_execution.md)) | Plans arrive one 10-tick window late; a fixed 100 ms PIBT reserve leaves short windows no guidance | 2-tick window + 20 ms reserve + plan-ahead (`default2026g`) | +32% on the competition setting, +27% on 2–5-stop tasks, +68–78% under breakdowns; **−16% at 300–400 robots and −32% on 5-stop tours**: a 60 ms window is too short there |
| The 2026 kit gains least from multi-stop tasks (A3) | Goals are frozen within a window, so each errand change waits for a plan | Plan-ahead only (`default2026nx`, `--plan-ahead`) | 5-stop tours +26% (0.124 → 0.156); +3% on the competition setting; 300 robots +5%, 400 robots −8% |
| Kitty_Knight crashes under any fault ([B1](B1_breakdowns.md)) | Exits when execution differs from its plan | Resync from the actual states (`kitty2024rs`, `--kitty-resync`) | Survives; 0.61 at a breakdown rate of 0.001 |

## Proposed next (not built)

1. **Window sized to the planner's cost** (2026). The best window is the shortest whose planning finishes in time, and it grows with the fleet and the number of errands per task: choose it per instance, or adapt it online from the measured planning time.
2. **Speed-aware planning** for slow robots: plan them at half speed instead of as repeatedly stopped (B1 verdict 3).
3. **Closure-aware heuristics** (C1): recompute distance tables for the cells around a closed aisle while it is closed; the fault-aware build does not help here (−3/−4%).
4. **An age term in the scheduler** (A1, E2's far task): distance-greedy schedulers never serve a far task while nearer ones keep arriving. With urgent deadlines (D = 100), least-slack does not beat oldest-first because the 2026 planner's latency (p50 145–276 ticks), not the task order, sets the misses; No Man's Sky, the fastest planner (p50 ≈ 80), misses the fewest urgent tasks (38–43%). Deadline-aware scheduling needs a fast planner first.
5. **Fault-aware No Man's Sky and 2026 planners**: the same pinning in their PIBT; for the 2026 kit, the planner must also be told about breakdowns that happen during the window.
6. **Kitty_Knight on warehouse maps with Poisson demand** runs out of memory or finishes nothing (A2): open.
