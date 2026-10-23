# Stage 1: the algorithms, their pros and cons, and what they predict

This is a code-level comparison of the six solvers in the benchmark. [`../explainer.md`](../explainer.md) §3 and [`../planning.md`](../planning.md) explain *how* each one works; this document asks *what each one assumes*, what that buys, what it costs, and which weaknesses that predicts. The predictions at the end (P1–P12) are tested in the later stages.

Code links point to the pinned upstream sources: the [Start-Kit v3.1.0](https://github.com/MAPF-Competition/Start-Kit/tree/v3.1.0) for the 2026 kit, and the [Code-Archive](https://github.com/MAPF-Competition/Code-Archive/tree/main/2024%20Competition) for the 2024 entries (`scripts/setup.sh` pins the commits).

[SK]: https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0
[NMS]: https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc
[KK]: https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_Kitty_Knight/da7d428b40cc1425b381adefe9709d4accbfdfe1
[D24]: https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_Default_Entry/5ea4fa167908e5bfe53a7f2a768f5d12dccf3c5a

## 1. The design space

Every solver answers the same eight questions. The table is the whole comparison in one place; the sections below justify each cell.

| Decision | 2026 default (+ oldest-first, least-slack) | 2024 default | No Man's Sky (2024 1st) | Kitty_Knight (2024 2nd) |
|---|---|---|---|---|
| **Who gets which task** | Greedy, robot by robot: minimise *distance to first errand + task length* (oldest-first / least-slack replace this) | Same greedy | Closest-pair matching, then simulated annealing on Σ (5·distance + task length) | Closest-pair matching on distance + task length |
| **Can a task change robot?** | Never | Never | Every tick, until its first errand is done | Every tick, until its first errand is done |
| **Planning horizon** | PIBT rolled out for a whole window (≈11 ticks), goals frozen during it | 1 tick | 5 ticks (fixed 5-action macros), re-planned every tick | 20-tick windowed plan, extended and repaired every tick, 1 tick executed |
| **Turning** | Ignored by PIBT, added afterwards as extra ticks | Same | Part of the search (macros contain turns) | Part of the search (SIPP over (cell, heading)) |
| **Collision resolution** | PIBT priority inheritance + a cell-visiting-order executor (TPG) | PIBT priority inheritance | Many randomized PIBT-style searches over macros, best score kept | Prioritized planning + SIPP inside large-neighbourhood search, LaCAM2 for the initial window |
| **Congestion avoidance** | Traffic-flow guide paths (Frank-Wolfe) | Same | Hand-drawn one-way lanes on the competition random map; a dynamic heuristic elsewhere | Learned edge weights for specific fleet sizes |
| **Idle robots** | Drive back to their start cell, lowest priority | Same: drive back to their start cell | Stand still, zero weight in the optimiser ("stand still and don't bother anyone") | Goal = current cell |
| **Compute** | 1 thread | 1 thread | 32 threads | OpenMP + threads |
| **Assumes** | the real-time 2026 loop | always-full task pool | competition map *by file name*, fleet size *by exact count*, always-full pool | competition map *by file name*, fleet size, always-full pool, *perfect execution* |

## 2. The shared core: PIBT on robots that turn

All four families start from PIBT (Okumura et al., AIJ 2022): each tick, robots in priority order pick their best neighbouring cell, and a robot in the way inherits the priority and must move first ([`causalPIBT`][SK-pibt]).

[SK-pibt]: https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/default_planner/pibt.cpp#L44

**Pros.**
- Linear-time per tick, so it scales to thousands of robots.
- Never deadlocks *if* every robot's goal is reachable and the graph is biconnected (the reachability guarantee of PIBT); in practice, livelocks are rare.
- Purely reactive: it re-reads the world every tick, so it tolerates surprises (a robot that didn't move is simply where it is).

**Cons, as implemented here.**
- **It ignores heading.** The default planners choose a *cell*; a robot not facing it spends 1–2 ticks turning while the cell stays reserved ([`getAction`][SK-pibt]). A U-turn costs two ticks of pure waiting, and a robot behind a turning robot waits too, because forward moves only happen if the robot ahead also moves forward this tick ([`moveCheck`](https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/default_planner/pibt.cpp#L184)).
- **One-step greed.** A robot never accepts a worse cell now for a better path later, except through the guide path. Head-on meetings in corridors are resolved by whoever has priority pushing the other back, possibly far (see the *idle blocker* scene, where both defaults push an idle robot down the whole corridor).
- **Priority = how long you have been trying.** A robot's priority grows by 1 per round while its goal is unchanged, +10 in a dead end ([`planner.cpp#L157-L160`](https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/default_planner/planner.cpp#L157-L160)). Good for eventually unblocking, but nothing in it knows about deadlines, task value or a robot that *cannot* move.

## 3. The 2026 default (and the two baseline schedulers on its planner)

**How it differs from the 2024 default.** The scheduler, PIBT and the traffic-flow guidance are the same code (diffing the two trees shows only cosmetic changes). What changed is *execution*:

1. The planner runs on a background thread. It gets a snapshot of the predicted state, plans a window of ticks, and the result is handed to the robots one window later ([`CompetitionSystem.cpp#L184`](https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/src/CompetitionSystem.cpp#L184)).
2. Inside a window, PIBT is rolled forward step by step with **goals frozen**: a robot that reaches its goal during the rollout stays there until the next planning call gives it the next errand ([`plan`](https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/default_planner/planner.cpp#L343)).
3. The executor stages only the moves (planned waits are dropped), enforces the planned order of visits to each cell (a temporal plan graph, TPG), and emits GO/STOP per robot per tick ([`default_executor.cpp`](https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/default_planner/default_executor.cpp#L27), [`mcp`](https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/default_planner/default_executor.cpp#L213)).

**Pros.**
- **Robust execution.** The TPG executor tolerates delays: a delayed robot makes the robots that depend on it STOP, nobody collides, and the plan resumes ([*delay convoy* scene](../scenes.md#delay-convoy)). This is the only solver designed for imperfect execution.
- **Real-time decoupling.** Planning time never stops the robots; a slow planner degrades gracefully instead of freezing the fleet.
- **Readable baselines.** One thread, simple rules: a clean reference for scheduler experiments (oldest-first, least-slack plug in without touching the planner).

**Cons.**
- **Latency on every goal change.** A new task, or the next errand of a task, reaches the robot only in the plan computed *after* the goal changed, which arrives one window later. With a 10-tick window, every errand transition costs up to ~2 windows of standing still; in the *idle blocker* scene the robot waits 20 ticks before its first move. The README benchmark shows busy robots waiting on 50% of ticks.
- **Turns as afterthought, twice.** Planned without turns, then executed in a window where each turn delays everything after it in the TPG order.
- **Traffic guidance needs time it may not get.** Guidance gets the window minus at least 100 ms reserved for PIBT ([`planner.cpp#L362`](https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/default_planner/planner.cpp#L362), [`MIN_PIBT_TIME`](https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/default_planner/const.h#L12)); with short windows it gets none.
- **Idle robots drive home.** An idle robot's placeholder goal is its *start cell* ([`planner.cpp#L99`](https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/default_planner/planner.cpp#L99)), so between tasks it drives across the map to park, adding traffic and moving away from where work appears.

**The greedy scheduler.** Each free robot, in arbitrary (hash-set) order, takes the open task minimising *distance to the first errand + the task's own errand-to-errand length* ([`scheduler.cpp#L68`](https://github.com/MAPF-Competition/Start-Kit/blob/v3.1.0/default_planner/scheduler.cpp#L68)).
- Pro: little empty driving; cheap; never reassigns, so no wasted half-trips.
- Con: **it prefers short tasks**, not just near ones. Long tasks are never anyone's best choice while short ones exist: in pool mode they wait longest (its pool p99 is 798 ticks vs 514 for oldest-first). Commits forever, so a robot that gets stuck keeps its task. Blind to deadlines, service times, and to robots about to become free nearby.

**Oldest-first** fixes starvation (no task waits forever) at the cost of criss-crossing (*scattered batch* scene). **Least-slack** reads deadlines and service-time quantiles, but under overload it serves the most urgent first and makes *more* tasks late (*slow station* scene).

## 4. The 2024 default

Same scheduler and planner as the 2026 default, executed one tick at a time in the planner-paced 2024 kit: the planner sees the true state and new goals every tick. In the scenes and the benchmark it gets roughly twice the 2026 default's throughput, which isolates the cost of the 2026 execution model (question E1). Two further differences:

- **Idle robots drive back to their start cell too** (the placeholder goal is the robot's cell at tick 0, [`planner.cpp#L96`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_Default_Entry/5ea4fa167908e5bfe53a7f2a768f5d12dccf3c5a/default_planner/planner.cpp#L96)), and park there with the lowest priority. In pool mode robots are never idle, so this never shows in the competition. At low load, parked robots box busy ones in: 82% of the stuck time of late tasks has an idle robot adjacent (README finding 3).
- **The 2024 kit punishes any invalid move by making every robot wait** ([`Simulator.cpp#L25`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_Default_Entry/5ea4fa167908e5bfe53a7f2a768f5d12dccf3c5a/src/Simulator.cpp#L25)). Planners never produce invalid moves when execution is perfect, so this rule is invisible in the competition — and fatal for any model of imperfect execution (see §7).

## 5. No Man's Sky (2024 1st)

**Planner.** Each robot chooses one of 129 fixed 5-tick action sequences (macros of wait / turn right / turn left / forward, [`operations.cpp#L71`][NMS-ops]). 32 threads each build a joint choice PIBT-style and then improve it by simulated annealing for the rest of the time budget; the best-scoring joint choice wins and only its first action is executed ([`eplanner.cpp#L50`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Planner/eplanner.cpp#L50), [`PIBTS::solve`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Planner/PIBT/pibts.cpp#L920)). The score is a priority-weighted sum of how much closer each robot gets to its target.

[NMS-ops]: https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Objects/Environment/operations.cpp#L71

**Scheduler.** Closest-pair matching, then simulated annealing on the total cost Σ (5 · distance to first errand + task length), re-run every tick over every task whose first errand isn't done yet, so tasks move between robots freely ([`scheduler_solver.cpp#L63-L75`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Scheduler/scheduler_solver.cpp#L63-L75), [`#L194`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Scheduler/scheduler_solver.cpp#L194)).

**Pros.**
- **Turns are planned, not patched.** Macros include turns, so robots turn early and move in convoys; it has the highest move share and the lowest travel stretch (1.4× the map distance vs 3.0× for the 2026 default, explainer §3.7).
- **Anytime and parallel.** More cores or time → more annealing → better plans; at worst it returns the PIBT construction.
- **Re-assignment** fixes bad matches as robots free up (*re-assignment* scene: 10 ticks faster).
- **Penalising empty driving 5×** favours short pickups, which matters when robots are the bottleneck.

**Cons.**
- **Tuned to the exact competition instances.** It identifies the map *by file name*: only the five competition map names are recognised, anything else is `NONE` ([`environment.cpp#L120`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Objects/Environment/environment.cpp#L120)). Then:
  - the hand-drawn one-way lane file for `random-32-32-20` is loaded only for that name ([`guidance_map.cpp#L6`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Objects/Environment/guidance_map.cpp#L6));
  - the 5× empty-driving weight applies only to recognised maps; on any other map the scheduler cost is plain distance + length;
  - the test type is set by *exact* robot count (100, 200, 400, 700, 800, [`environment.cpp#L220`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Objects/Environment/environment.cpp#L220)), which selects search depth, priority shaping, and how many robots may work at all.
- **It deliberately idles part of the fleet on dense instances.** `DISABLE_AGENTS` caps the robots allowed to hold a target (400 of 700 on the random map, 2,500 of 6,500 on the game map, [`info.cpp#L23`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Objects/Environment/info.cpp#L23), [`robot_handler.cpp#L93`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Objects/Environment/robot_handler.cpp#L93)). The cap is a constant per competition instance, not something it adapts.
- **Idle robots are immovable.** A robot without a target gets zero weight and prefers to wait ([`pibts.cpp#L181`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Planner/PIBT/pibts.cpp#L181), [`#L708`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Planner/PIBT/pibts.cpp#L708)). Parked on someone's goal, it stays there (*two doors* scene).
- **It can livelock.** On the *far task* scene all four robots orbit their goals in a 12-tick cycle from tick 84 to the end (reproducible; cause open, question E2).
- **Starvation by design.** It minimises total cost, with no age or deadline term; a far task is never worth it while near ones keep arriving (*far task* scene: never served).
- **Compute-hungry.** 32 threads ([`settings.hpp#L57`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/settings.hpp#L57)); on our 2 pinned cores each gets 1/16 of a core. On this map it loses nothing from that (README caveat), but on bigger instances it would.

## 6. Kitty_Knight (2024 2nd, WPPL)

**Planner.** Keeps a 20-tick plan for every robot. Each tick it extends short plans with LaCAM2 (a fast PIBT-based search), then runs large-neighbourhood search (LNS): destroy 8 robots' paths, re-plan them one by one with SIPP against everyone else's (prioritized planning), keep the change if the total cost drops. Executes one tick ([`LNSSolver.cpp`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_Kitty_Knight/da7d428b40cc1425b381adefe9709d4accbfdfe1/wppl_planner/src/LNS/LNSSolver.cpp#L95)). Configuration per map file and per fleet size: windows, learned edge weights for 200, 500 and 700 robots ([config `#L33`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_Kitty_Knight/da7d428b40cc1425b381adefe9709d4accbfdfe1/wppl_planner/configs/random-32-32-20.json#L33)), and above 700 robots a different team's planner entirely (["Shadocks"](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_Kitty_Knight/da7d428b40cc1425b381adefe9709d4accbfdfe1/wppl_planner/configs/random-32-32-20.json#L23)).

**Scheduler.** Closest-pair greedy matching over every robot and every unopened task, cost = distance + task length, re-run every tick ([`scheduler.cpp#L368`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_Kitty_Knight/da7d428b40cc1425b381adefe9709d4accbfdfe1/wppl_planner/src/scheduler.cpp#L368)).

**Pros.**
- **Best plan quality per tick when the world matches the plan.** Windowed, rotation-aware, collision-free paths that LNS keeps improving: the highest pool throughput in our benchmark (1.11 tasks/tick).
- **Global matching** is better than robot-by-robot greedy: the closest pair overall goes first.

**Cons.**
- **Assumes there are always more tasks than robots.** The scheduler calls `exit(-1)` otherwise ([`scheduler.cpp#L206`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_Kitty_Knight/da7d428b40cc1425b381adefe9709d4accbfdfe1/wppl_planner/src/scheduler.cpp#L206)); our guard keeps the first *n* robots by index and idles the rest, an arbitrary choice.
- **Assumes perfect execution.** It advances its plan only when *every* robot reached its planned next state ([`#L401`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_Kitty_Knight/da7d428b40cc1425b381adefe9709d4accbfdfe1/wppl_planner/src/LNS/LNSSolver.cpp#L401)), and if some moved while others didn't, it exits ([`#L490`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_Kitty_Knight/da7d428b40cc1425b381adefe9709d4accbfdfe1/wppl_planner/src/LNS/LNSSolver.cpp#L490)). Any delay, breakdown or slow robot breaks it.
- **Idle robots hold their cell** (goal = current cell), worst of all at low load: 89% of late tasks' stuck time has an idle robot adjacent (README finding 3), and it never gets under 1% misses at any tested rate.
- **Heavy and brittle on finite workloads.** It stalls and crashes when tasks run out (scenes), and its LNS loops forever looking for 8 robots to destroy in teams smaller than 9 (handled in the scene harness).

## 7. Hidden assumptions, side by side

The competition hides these assumptions because its setting satisfies all of them. Each research question in [`README.md`](README.md) breaks one.

| Assumption | 2026 / baselines | 2024 default | No Man's Sky | Kitty_Knight | Broken by |
|---|---|---|---|---|---|
| More tasks than robots, always | ok | idle robots block | idle robots immovable | crashes (guarded) | D2, A1–A4 at low load |
| Execution is perfect | tolerant (TPG) | the kit's all-wait rule | the kit's all-wait rule | exits on mismatch | B1, B2, B3, C1 |
| Map is static | guide paths precomputed | same | heuristics precomputed | same | C1 |
| Map is a competition map, named as such | no tuning | no tuning | lanes, weights, depth by name | config by name | C2 |
| Fleet size is a competition size | no tuning | no tuning | test type by exact count | weights/windows by count | C3 |
| Tasks are interchangeable; only the count matters | greedy prefers short | same | minimises total cost | same | A1, A3, A4 |
| No deadlines | least-slack only | none | none | none | A1 |
| Lots of compute | 1 thread | 1 thread | 32 threads | multi-threaded | C3 |

## 8. Predictions to test

Written before running the stage 2–3 benchmarks.

| Id | Prediction | Tested in |
|---|---|---|
| P1 | No Man's Sky loses a large share of its lead when the random map is loaded under a different file name (no lanes, no 5× weight, no test type). | C2 |
| P2 | Kitty_Knight's and No Man's Sky's leads shrink on warehouse and sortation maps; the defaults' guided PIBT suffers in narrow aisles. | C2 |
| P3 | Under breakdowns with a local safety stop, Kitty_Knight crashes unless re-synchronised; PIBT-based planners keep pushing the broken robot and jam behind it. | B1 |
| P4 | Under the 2024 kit's all-wait rule, one broken robot freezes the whole fleet for every planner that keeps commanding it. | B1 |
| P5 | The 2026 executor degrades most gracefully under breakdowns and slow robots, but its plan latency still costs throughput. | B1, B2 |
| P6 | With per-task deadlines, only least-slack protects urgent tasks; greedy and annealing miss far urgent tasks at the same rate as standard ones. | A1 |
| P7 | Station-based warehouse tasks create queues at stations that distance-only schedulers ignore; robots queue in aisles and block through-traffic. | A2 |
| P8 | Greedy's short-task preference makes long tasks (tours, long trips) wait disproportionately; oldest-first evens this out. | A3, A4 |
| P9 | Throughput per robot falls as the fleet grows; for No Man's Sky the per-count tuning makes the curve non-smooth (best at exactly 100/200/400). | C3 |
| P10 | The 2026 default's gap to the 2024 default closes when its planning window shrinks to one tick. | E1 |
| P11 | Closing an aisle costs every planner, because precomputed heuristics keep routing robots into the closure. | C1 |
| P12 | Where idle robots wait matters: driving home (both defaults) adds traffic and leaves robots far from new work; staying put (Kitty_Knight, No Man's Sky) leaves them in aisles. A parking policy that sends idle robots to off-aisle cells near expected demand beats both. | D2, A2 |
