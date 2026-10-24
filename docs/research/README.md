# Research programme: how lifelong MAPF solvers break when the problem gets realistic

The League of Robot Runners (LoRR) scores solvers on one thing: tasks finished per tick, with an always-full task pool, robots that never fail, identical errands, and maps the entries were tuned for. Real fleets (warehouses, sortation centres, chain-store logistics, autonomous vehicles) differ on almost every one of those points. This programme asks which solver weaknesses those differences expose, *why* they occur, and how to fix them.

It builds on the rest of the repo:
- [`../explainer.md`](../explainer.md): the problem, environment, schedulers, planners and metrics.
- [`../planning.md`](../planning.md): the planners in depth.
- [`../scenes.md`](../scenes.md): nine small situations that isolate one behaviour each.
- The README's benchmark: capacity and deadline metrics under bursty arrivals and service times.

## The five stages

| # | Stage | Output |
|---|---|---|
| 1 | Understand the algorithms: design choices, assumptions, pros and cons, and the weaknesses they *predict* | [`algorithms.md`](algorithms.md) |
| 2 | Benchmark them on the competition setting and on realistic maps, to find weaknesses | [`baseline.md`](baseline.md) |
| 3 | Change the problem (the research questions below), benchmark every solver against each change | one report per question in this folder |
| 4 | For each weakness, a root-cause hypothesis, and an experiment that confirms or refutes it | the "Why" section of each report |
| 5 | Proposals to overcome each weakness, prototyped and measured where feasible | the "Fix" section of each report, summarised in [`proposals.md`](proposals.md) |

Each report follows the same shape: **question → real-world motivation → how it is modelled → results → why (hypothesis, test, verdict) → fix**. Negative results are reported as such.

## Solvers

| Id | Solver | Kit |
|---|---|---|
| `default2026` | 2026 default: greedy scheduler, traffic-guided PIBT, TPG executor | 2026 (real-time) |
| `fifo2026` | oldest-first scheduler on the 2026 default planner | 2026 |
| `slack2026` | least-slack scheduler on the 2026 default planner | 2026 |
| `default2024` | 2024 default: greedy + guided PIBT | 2024 (planner-paced) |
| `nms2024` | No Man's Sky (2024 1st): annealing scheduler + 32 randomized PIBTs | 2024 |
| `kitty2024` | Kitty_Knight (2024 2nd, guarded): greedy matching + windowed LNS (WPPL) | 2024 |

## Research questions

Each question changes one thing about the competition problem. The *prediction* column is written before running anything, from the algorithm analysis in stage 1, so the benchmark can confirm or refute it.

### A. Tasks are not all alike

| Id | Question | Real-world case | Model | Prediction |
|---|---|---|---|---|
| A1 | What if each task has its own deadline, and some are urgent? | Express vs standard orders; same-day delivery; a passenger's maximum pickup wait | Deadline classes by task id (e.g. 20% at D = 100, 80% at D = 400); misses reported per class | Only least-slack reads deadlines. Greedy and annealing will miss urgent tasks that are far away at the same rate as standard ones. |
| A2 | What if work concentrates at a few stations? | Goods-to-person picking: every task ends at one of a few pick stations, which also take time to serve | Warehouse map; errands shelf → station; station service times | Queues at stations; distance-greedy schedulers ignore the queue, so robots pile up in front of busy stations and block the aisles. |
| A3 | What if tasks are long multi-stop tours? | Chain-store replenishment: one truck/robot visits several shops in a row | Tasks with 5–8 errands, service at each stop | Assignment matters less (most time is in the tour); switching an opened task is impossible, so a stuck robot holds a long task hostage. |
| A4 | What if tasks are pickup-dropoff trips across a large area? | Ride-hailing / AV fleets, taxis | 2-errand tasks with long, random trips; deadline on the pickup | Empty driving dominates; schedulers that weigh it (No Man's Sky's 5×) gain; greedy's far-task starvation shows up as long passenger waits. |

### B. Robots are not perfect

| Id | Question | Real-world case | Model | Prediction |
|---|---|---|---|---|
| B1 | What if robots break down and stay put for a while? | Mechanical faults, e-stops, a human blocking a robot | Each robot fails with a small probability per tick and stays frozen for a repair time (MTBF/MTTR); the simulator applies a local safety stop | Planners treat a frozen robot as movable. PIBT keeps trying to push it, so traffic behind it jams; schedulers keep its task (locked once opened). Windowed planners (WPPL) and the 2026 executor jam longest. |
| B2 | What if some robots are permanently slower? | Mixed fleet generations, heavier loads | A share of robots can only move every other tick | Plans assume equal speed; fast robots queue behind slow ones in narrow aisles; multi-step plans (2026, WPPL) desynchronise. |
| B3 | What if robots must stop to recharge? | Battery-powered AMRs | Robots become unavailable periodically, near-deterministically, at their current cell | Like B1 but predictable: a solver that knew the schedule could route around; none does. |

### C. The environment is not fixed

| Id | Question | Real-world case | Model | Prediction |
|---|---|---|---|---|
| C1 | What if an aisle is closed for a while? | A spill, maintenance, a human picker zone | Cells that robots may not enter during an interval; planners are not told | Heuristics are precomputed on the static map, so every planner keeps routing into the closure and waits at its edge. |
| C2 | Does the ranking survive a different map layout? | Fulfilment warehouse, sortation centre | `warehouse_small`, `sortation_small` (same size class as the benchmark map) | No Man's Sky's hand-made lane map is for the competition maps only; its lead may shrink. Narrow aisles hurt planners that cannot coordinate head-on traffic. |
| C3 | What happens as the fleet grows? | Scaling a site from 100 to 400 robots | Official 100/200/400-robot instances on the same map | Throughput saturates, then falls; the per-robot compute budget shrinks for search-based planners (WPPL) first. |

### D. Demand is not steady (partly covered by the README benchmark)

| Id | Question | Real-world case | Model | Prediction |
|---|---|---|---|---|
| D1 | What if demand comes in waves? | Truck arrivals, lunch rush | MMPP / Hawkes arrivals (existing) | Done in the README benchmark: burstiness costs even the winner a third of its capacity. Revisited only on new maps. |
| D2 | What if demand drops to nearly nothing? | Night shift | Low Poisson rates (existing) | Done: idle robots block busy ones (Kitty_Knight, 2024 default). Revisited for the root cause in stage 4. |

### E. Open threads from earlier work

| Id | Question | Status |
|---|---|---|
| E1 | Why is the 2026 kit's planner productive only about half as much as the 2024 default, even once its plan latency is removed? | Plan latency explains part (see [`../scenes.md`](../scenes.md)); the rest is unexplained. |
| E2 | Why does No Man's Sky livelock on the `far_task` scene? | Reproducible from tick 84; cause unknown. |

## Status

| Stage / question | Status |
|---|---|
| 1 Algorithms | done ([algorithms.md](algorithms.md)) |
| 2 Baseline | done ([results.md](results.md)) |
| A1–A4, B1–B3, C1–C3 | done ([results.md](results.md)); E1, E2 root-caused and fixed |
| E1, E2 | planned |
