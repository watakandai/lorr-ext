# B1–B3: why a 2% breakdown rate costs 20–30% of throughput, and a planner fix

**Question.** With breakdowns at a rate of 0.0002 per robot per tick and repairs of 50–150 ticks, a robot is frozen about 2% of the time. Yet the 2024 default loses 19% of its throughput, No Man's Sky 17% and the 2026 default 27% ([results.md](results.md)). Where does a tenfold amplification come from?

**Hypothesis.** No planner is told which robots are frozen. Every tick the 2024 default plans a broken robot as moving, and plans the robots behind it into the cell it "vacates". The simulator's local safety stop then cancels those moves as well, so one broken robot stalls a queue. The grid's counters support this: in the 2024 kit, *cascade waits* (robots stopped because their target cell is held by a frozen robot) are 2.7–3.7× the frozen time itself. With the 2024 kit's all-stop rule (any invalid move stops every robot), the same planning error stops the whole fleet: 0.03 tasks/tick at a rate of 0.001.

**Prediction.** A planner that knows which robots are stopped, and plans them as obstacles, removes the cascade waits and most of the extra loss; under the all-stop rule it should make that rule harmless, because no robot is planned into a frozen one any more.

## Experiment

`default2024fa` (`tools/patch_kit.py --fault-aware`): the 2024 default planner, told each tick which robots are frozen (as a fleet manager knows which robots reported a fault). Before PIBT runs it pins them to their cells and cancels every move another robot had committed into a pinned cell, transitively along chains of robots behind it. Everything else is unchanged. Run by `python -m lorrext.research_fixes` on every B1/B2/B3/C1 setting; mean of 2 seeds.

| Fault | 2024 default | + fault-aware | Gain | Cascade waits (before → after) |
|---|---|---|---|---|
| none | 0.812 | 0.810 | 0% | — |
| breakdowns 0.0002 | 0.655 | 0.744 | +14% | 4821 → 2 |
| breakdowns 0.0005 | 0.544 | 0.690 | +27% | 10444 → 2 |
| breakdowns 0.001 | 0.435 | 0.592 | +36% | 14160 → 6 |
| breakdowns 0.002 | 0.271 | 0.492 | **+82%** | 19960 → 13 |
| breakdowns 0.0002 + all-stop rule | 0.234 | 0.752 | ×3.2 | 924 → 2 |
| breakdowns 0.001 + all-stop rule | 0.028 | 0.593 | ×21 | 1381 → 7 |
| pauses (10% of time, 20-tick blocks) | 0.599 | 0.670 | +12% | 6834 → 0 |
| 10% slow robots | 0.767 | 0.767 | 0% | 819 → 0 |
| 30% slow robots | 0.635 | 0.432 | **−32%** | 3496 → 0 |
| 50% slow robots | 0.533 | 0.348 | **−35%** | 4333 → 0 |
| aisle closure (8 / 16 cells) | 0.627 / 0.584 | 0.603 / 0.568 | −4% / −3% | not robot faults: unchanged |

For comparison, No Man's Sky, the strongest original solver, gets 0.347 at a breakdown rate of 0.002: the fault-aware 2024 default beats it by 42% there.

## Verdict

1. **Confirmed for breakdowns.** Cascade waits disappear and between half and all of the extra loss comes back; the gain grows with the breakdown rate. At 0.0002 the fault-aware planner loses 8% (0.81 → 0.74), about 4× the frozen time; the rest is real blocking: a broken robot still occupies a cell, often in a corridor, and its task waits for the repair.
2. **The all-stop rule is a symptom of the planner's ignorance, not a cost of its own.** With fault-aware planning no robot is ever planned into a frozen one, so the rule almost never fires (2–6 all-stops per run) and throughput equals the local-stop result.
3. **Wrong for one-tick stops.** A slow robot is frozen every other tick. Pinning it on its off ticks and cancelling the commitments behind it each time makes the robots around it re-plan every tick and lose PIBT's multi-tick turn commitments; throughput drops by a third. Runs also vary more (50% slow, seed 0: 0.41 and 0.52 on two runs; the planner is time-bounded). A slow robot must be modelled as slow (half speed in the plan), not as repeatedly stopped.
4. **Bugs found on the way.** The first two versions of the patch pinned the frozen robot without cancelling the moves committed into its cell (or skipped robots that had a committed move, which includes almost every broken robot); with slow robots two robots then swapped cells, an edge conflict, and the 2024 kit stopped every robot for the rest of the run. The cancellation must follow chains of robots, which is why the patch iterates to a fixed point.

## Other solvers

- **Kitty_Knight** crashes on every fault type: it exits when execution differs from its plan. `kitty2024rs` (`--kitty-resync`: restart from the actual states instead of exiting) survives, at 0.61 with a breakdown rate of 0.001.
- **No Man's Sky** has the same cascade problem (7046 cascade waits at 0.0002); the same pinning would apply to its PIBT, not built.
- **2026 kit:** its executor already applies a local stop, so cascade waits are 0, but robots wait on the frozen robot through the temporal plan graph instead; the plan is computed one window late, so the planner cannot even see a fresh breakdown. The 2-tick window build (`lorr2026g`, [E1](E1_2026_execution.md)) gains 68–78% under breakdowns, more than without faults (+32%): fresher plans see breakdowns sooner.

## Fix

Give the planner the fault state: pin robots that are stopped for more than a tick, release everything committed into them, and model slow robots as slow. See [proposals.md](proposals.md).
