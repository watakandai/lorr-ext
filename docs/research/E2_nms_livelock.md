# E2: why No Man's Sky livelocks, and a one-line fix that also raises its competition score

**Question.** On the [*far task* scene](../scenes.md#far-task), No Man's Sky (the 2024 winner) finishes 16 tasks by tick 80 and then nothing at all: from tick 84 its four robots repeat a 12-tick loop to the end of the run. Why?

## Result in one table

| | Original | Fixed (one rule, see below) |
|---|---|---|
| One robot alone, goal one step away diagonally | never arrives (12-tick loop) | arrives at tick 6 |
| *far task* scene, tasks finished of 101 | 16, then livelock | **83** |
| Competition benchmark (random-32-32-20, 100 robots, pool), tasks/tick | 1.064 | **1.134 (+6.6%)** |
| Same, p99 latency | 583 | 536 |

## Narrowing it down

1. **The loop is deterministic.** Three reruns of the scene enter the same 12-tick cycle at tick 84; all four robots hold *opened* tasks (first errand done, so the scheduler can no longer move them), and each robot circles its next errand cell at distance one, never stepping on it, although the cell is free.
2. **It needs no other robot.** Restarting from the tick-84 positions with only the pending goals, every robot *alone* falls into the same loop by tick 3 (`lorrext/scenes.py` scenes `e2_only0..3`). So it is not a multi-robot conflict: it is in how one robot scores its moves.
3. **Instrumented build.** A copy of the entry printing, each tick, the chosen 5-tick macro and the score of every macro that reaches the target shows, for the robot at (3,6) facing south with the goal at (3,7):

   | Macro | What it does | Score (lower wins) |
   |---|---|---|
   | `CFWWW` (index 29) | turn east, step onto the goal, wait | −200 − 29 = **−229** |
   | `FCFCF` (index 87) | go south, turn, east, turn, north onto the goal at step 5 | −200 − 87 = **−287** (chosen) |

   Only the first action of the chosen macro is executed (go south); next tick the robot is again one step from the goal, and again prefers the macro that reaches it at the horizon's end. The arrival keeps receding.

## Root cause

[`pibts.cpp#L171-L177`](https://github.com/MAPF-Competition/Code-Archive/blob/main/2024%20Competition/Team_No_Man's_Sky/44950d1b61b7482b4529a9dc127fa9d220962bdc/Solution/Planner/PIBT/pibts.cpp#L170-L176):

```cpp
// [KEK]: if we pass through the target, we should do it as early as possible. 7200 -> 7297
for (uint32_t d = 0; d < DEPTH; d++) {
    if (get_graph().get_pos(path[d]).get_pos() == target) {
        dist = d;
        dist = -dist;
    }
}
dist = dist * 50 - desired;
```

The comment says *as early as possible*; the code does the opposite twice:
- it keeps the **last** step at which the macro is on the target (the loop has no `break`), and
- it scores that step as `-d`, so **later is better** (lower score wins).

Every macro that is on the target at step 5 (whether it arrived at step 2 and waited, or looped around to arrive at step 5) ties at −200, and the tie is broken by `- desired`, the macro's index. Indices grow with the number of forward moves at the start, so the tie goes to the macro that moves away first. With a receding horizon that is a Zeno-style livelock: the robot is always 4 ticks from arriving.

**Why the competition never showed it.** It needs a free loop of exactly the right length around the goal. On the cluttered random map such loops are rare, and when the goal is straight ahead `FFFFW` (index 127) wins anyway; open rooms (the scenes) and open areas of warehouse maps provide them everywhere.

## Fix and verification

Score the **first** arrival and reward arriving earlier:

```cpp
for (uint32_t d = 0; d < DEPTH; d++) {
    if (get_graph().get_pos(path[d]).get_pos() == target) {
        dist = -static_cast<int64_t>(DEPTH - d);
        break;
    }
}
```

Built as a separate solver, `nms2024fix` (`tools/patch_kit.py --nms-arrival-fix`, see [`results.md`](results.md#fixes-stage-5)). It removes the loop in every reproduction and scene, and on the exact competition instance the authors tuned the rule on it *raises* throughput by 6.6% and lowers p99 latency by 8%: robots near their goal stop dawdling. The far task in the scene is still never served (max wait 200): that is a separate weakness, the scheduler's lack of any age term ([A1/A4](README.md)).
