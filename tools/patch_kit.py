#!/usr/bin/env python3
"""Apply the LoRR-Ext hooks to a Start-Kit tree (2026 kit or a 2024-season entry).

    python tools/patch_kit.py PATH/TO/KIT [--kitty-guard]

Idempotent: a tree that is already patched is left alone. Every hook is a
whitespace-tolerant regex, because some archived entries re-formatted the kit's
system files; a missing anchor aborts with an error instead of half-patching.
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path

EXT = Path(__file__).resolve().parent.parent / "ext"
MARK = "LORR_EXT_PATCHED"
FAULTS_MARK = "LORR_EXT_FAULTS"  # later hook set: applied on its own so built kits can be upgraded in place


def sub_once(text: str, pattern: str, repl, path: Path, what: str) -> str:
    new, n = re.subn(pattern, repl, text, count=1, flags=re.S)
    if n != 1:
        sys.exit(f"patch_kit: anchor for {what} not found in {path}")
    return new


def patch_file(path: Path, edits, mark: str = MARK) -> None:
    text = path.read_text()
    if mark in text:
        return
    for pattern, repl, what in edits:
        text = sub_once(text, pattern, repl, path, what)
    path.write_text(f"// {mark}\n" + text)


PLAN_AHEAD_HELPERS = r"""
#include "lorr_ext.h"
namespace DefaultPlanner {
    // LoRR-Ext plan-ahead (docs/research/E1_2026_execution.md): inside the multi-step
    // rollout, a robot that reaches its goal moves on to its task's next errand (after the
    // known 95th-percentile service time, if the cell has one) instead of waiting for the
    // next planning episode. Its guide path still ends at the old goal, so it uses the plain
    // distance heuristic to the new one.
    std::vector<char> lorrext_plain_h;
    static std::vector<int> lorrext_errand, lorrext_hold;
    static void lorrext_begin_rollout(SharedEnvironment* env)
    {
        lorrext_plain_h.assign(env->num_of_agents, 0);
        lorrext_errand.assign(env->num_of_agents, -1);
        lorrext_hold.assign(env->num_of_agents, -1);
        for (int i = 0; i < env->num_of_agents; i++)
        {
            int t = env->curr_task_schedule[i];
            if (t != -1 && env->task_pool.count(t))
                lorrext_errand[i] = env->task_pool[t].idx_next_loc;
        }
    }
    static void lorrext_advance_goals(SharedEnvironment* env, TrajLNS& lns)
    {
        for (int i = 0; i < env->num_of_agents; i++)
        {
            int t = env->curr_task_schedule[i];
            if (lorrext_errand[i] < 0 || env->goal_locations[i].empty() || !env->task_pool.count(t))
                continue;
            int g = env->goal_locations[i].front().first;
            if (env->start_states[i].location != g)
                continue;
            if (lorrext_hold[i] < 0)
                lorrext_hold[i] = LorrExt::service_quantile_at(g, 0.95);
            if (lorrext_hold[i] > 0) { lorrext_hold[i]--; continue; }
            const auto& task = env->task_pool[t];
            if (lorrext_errand[i] + 1 >= (int)task.locations.size())
                continue;
            int ng = task.locations[++lorrext_errand[i]];
            env->goal_locations[i].clear();
            env->goal_locations[i].push_back({ng, task.t_revealed});
            if (lns.heuristics[ng].empty())
                init_heuristic(lns.heuristics[ng], env, ng);
            lorrext_plain_h[i] = 1;
            lorrext_hold[i] = -1;
        }
    }
}
"""


def patch(kit: Path, kitty_guard: bool = False, stale_guard: bool = False, nms_arrival_fix: bool = False,
          plan_ahead: bool = False, pibt_reserve: int | None = None, kitty_resync: bool = False,
          fault_aware: bool = False) -> list[str]:
    done = []
    shutil.copy(EXT / "lorr_ext.h", kit / "inc" / "lorr_ext.h")

    patch_file(kit / "inc" / "TaskManager.h", [
        (r"(class\s+TaskManager\s*\{)", r"\1\n    friend struct LorrExt;", "friend decl"),
    ])
    patch_file(kit / "src" / "TaskManager.cpp", [
        (r"(#include\s+\"TaskManager.h\")", r'\1\n#include "lorr_ext.h"', "include"),
        # wrap the reveal loop (not the function start) so the 2024 kits still
        # run their `new_tasks.clear()` that precedes it
        (r"(while\s*\(\s*ongoing_tasks\s*\.\s*size\s*\(\s*\)\s*<\s*num_tasks_reveal\s*\))",
         r"if (!LorrExt::reveal(*this, timestep))\n    \1", "reveal hook"),
        (r"states\s*\[\s*k\s*\]\s*\.\s*location\s*==\s*ongoing_tasks\s*\[\s*current_assignment\s*\[\s*k\s*\]\s*\]\s*->\s*get_next_loc\s*\(\s*\)",
         "LorrExt::stop_reached(*this, k, states[k].location)", "service-time hook"),
    ])
    patch_file(kit / "src" / "driver.cpp", [
        (r"(#include\s+\"CompetitionSystem.h\")", r'\1\n#include "lorr_ext.h"', "include"),
        (r"(system_ptr\s*->\s*set_num_tasks_reveal\s*\([^;]*;)",
         r"\1\n    LorrExt::configure(data, team_size);", "configure hook"),
    ])
    patch_file(kit / "src" / "CompetitionSystem.cpp", [
        (r"(#include\s+\"CompetitionSystem.h\")", r'\1\n#include "lorr_ext.h"', "include"),
        (r"(js\s*\[\s*\"numTaskFinished\"\s*\]\s*=\s*task_manager\s*\.\s*num_of_task_finish\s*;)",
         r"\1\n    LorrExt::save(task_manager, js, simulation_time);", "save hook"),
    ])
    done.append("core hooks")

    # 2026 kit only: baseline schedulers + per-cell move delays
    sched = kit / "src" / "TaskScheduler.cpp"
    if (kit / "default_planner" / "scheduler.cpp").exists() and "planCommTimeLimit" in (kit / "src" / "driver.cpp").read_text():
        shutil.copy(EXT / "ext_scheduler.cpp", kit / "default_planner" / "ext_scheduler.cpp")
        patch_file(sched, [
            (r"(#include\s+\"TaskScheduler.h\")",
             r"\1\n#include <vector>\nclass SharedEnvironment;\nnamespace LorrExtSched { bool plan(int, std::vector<int>&, SharedEnvironment*); }",
             "include"),
            (r"(void\s+TaskScheduler::plan\s*\([^)]*\)\s*\{)",
             r"\1\n    if (LorrExtSched::plan(time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE, proposed_schedule, env)) return;",
             "scheduler hook"),
        ])
        patch_file(kit / "src" / "Simulator.cpp", [
            (r"(#include\s+\"Simulator.h\")", r'\1\n#include "lorr_ext.h"', "include"),
            (r"(delay_generator\s*->\s*nextTick\s*\(\s*\)\s*;)",
             r"if (LorrExt::simulate_delay(curr_states, num_of_agents)) return;\n    \1", "delay hook"),
        ])
        done.append("2026 baseline schedulers + per-cell delays")
        patch_file(kit / "src" / "Simulator.cpp", [
            (r"(\n(\s*)auto\s+pre_states\s*=\s*curr_states\s*;)",
             r"\n\2LorrExt::filter_actions(curr_states, actions, timestep);\1", "faults hook"),
        ], FAULTS_MARK)
    else:
        # 2024 kits: faults hook just before the joint action is validated
        patch_file(kit / "src" / "Simulator.cpp", [
            (r"(#include\s+\"Simulator.h\")", r'\1\n#include "lorr_ext.h"', "include"),
            (r"(\n(\s*)if\s*\(\s*!\s*model\s*->\s*is_valid\s*\(\s*curr_states\s*,\s*actions\s*,\s*timestep\s*\)\s*\))",
             r"\n\2LorrExt::filter_actions(curr_states, actions, timestep);\1", "faults hook"),
        ], FAULTS_MARK)
    done.append("robot faults hook")

    if stale_guard:
        # The archived 2024 default entry (kit v2.1.0) keeps a finished task id
        # in its proposed schedule until a free task exists to replace it, then
        # dereferences it. That never happens with an always-full pool but
        # crashes as soon as robots outnumber open tasks. Treat stale ids as
        # unassigned. Labelled "guarded" in results.
        patch_file(kit / "src" / "Entry.cpp", [
            (r"if\s*\(\s*t_id\s*==\s*-1\s*\)\s*continue\s*;",
             "if (t_id == -1 || env->task_pool.find(t_id) == env->task_pool.end() /* LoRR-Ext guard */\n"
             "            || env->task_pool[t_id].idx_next_loc >= (int)env->task_pool[t_id].locations.size())\n"
             "            continue;",
             "stale-task guard"),
        ])
        done.append("stale-task guard")

    if kitty_guard:
        # Kitty_Knight's scheduler calls exit(-1) when free robots outnumber
        # open tasks (it assumes the competition's always-full pool). Instead,
        # keep only as many assignable robots as there are tasks and leave the
        # rest idle this round. This changes the entry; its results are
        # labelled "guarded".
        n = 0
        for p in kit.rglob("scheduler.cpp"):
            if "build" in p.parts:
                continue
            text = p.read_text(errors="ignore")
            if "we always ensure more tasks than agents" not in text or MARK in text:
                continue
            text, k = re.subn(
                r"(std::cout\s*<<\s*\"error: we always ensure more tasks than agents\"[^;]*;\s*)exit\s*\(\s*-?\d+\s*\)\s*;",
                r"/* LoRR-Ext guard (was: print + exit(-1)) */ assignable_agents_indices.resize(assignable_tasks_indices.size());",
                text)
            if k:
                p.write_text(f"// {MARK}\n" + text)
                n += k
        # Its LNS also exit(-1)s when a robot sits short of its goal with a
        # path shorter than the conflict window, which service-time dwell
        # (robot pushed off a stop it is serving) produces. Downgrade those two
        # sanity checks to warnings; the simulator still validates every move.
        for p in kit.rglob("GlobalManager.cpp"):
            if "build" in p.parts or "wppl_planner" not in p.parts:
                continue
            text = p.read_text()
            if MARK in text:
                continue
            text, k = re.subn(r"(\"\s*vs\s*\"\s*<<\s*(?:window_size_for_PATH|instance\.goal_locations\[i\])\s*<<\s*endl\s*;\s*)exit\s*\(\s*-1\s*\)\s*;",
                              r"\1/* LoRR-Ext guard: was exit(-1) */", text)
            if k:
                p.write_text(f"// {MARK}\n" + text)
                n += k
        done.append(f"kitty guard ({n} sites)")
    if nms_arrival_fix:
        # No Man's Sky scores the LAST step a 5-tick macro spends on the target, as -d (later is
        # better), so a robot next to its goal loops to arrive at the horizon's end and never
        # arrives (docs/research/E2_nms_livelock.md). Score the first arrival, earlier = better.
        patch_file(kit / "Solution" / "Planner" / "PIBT" / "pibts.cpp", [
            (r"if\s*\(\s*get_graph\(\)\.get_pos\(path\[d\]\)\.get_pos\(\)\s*==\s*target\s*\)\s*\{\s*dist\s*=\s*d\s*;\s*dist\s*=\s*-dist\s*;\s*\}",
             "if (get_graph().get_pos(path[d]).get_pos() == target) {\n            dist = -static_cast<int64_t>(DEPTH - d); /* LoRR-Ext arrival fix */\n            break;\n        }",
             "nms arrival fix"),
        ], "LORR_EXT_NMS_ARRIVAL_FIX")
        done.append("nms arrival fix")
    if plan_ahead:
        dp = kit / "default_planner"
        patch_file(dp / "planner.cpp", [
            (r'(#include\s+"const.h"\s*\n)', lambda m: m.group(1) + PLAN_AHEAD_HELPERS, "helpers"),
            (r"(const\s+std::vector<DCR>\s+original_decided\s*=\s*decided\s*;)",
             r"\1\n        const auto lorrext_original_goals = env->goal_locations;\n        lorrext_begin_rollout(env);", "begin"),
            (r"(append_actions_and_rollout_states\(env,\s*actions,\s*one_step_actions\)\s*;)",
             r"\1\n            lorrext_advance_goals(env, trajLNS);", "advance"),
            (r"(\n(\s*)decided\s*=\s*original_decided\s*;)",
             r"\1\n\2env->goal_locations = lorrext_original_goals;\n\2lorrext_plain_h.assign(env->num_of_agents, 0);", "restore"),
        ], "LORR_EXT_PLAN_AHEAD")
        patch_file(dp / "pibt.cpp", [
            (r"(int\s+get_gp_h\s*\(\s*TrajLNS&\s*lns\s*,\s*int\s+ai\s*,\s*int\s+target\s*\)\s*\{)",
             r"extern std::vector<char> lorrext_plain_h;\n\1\n    if (ai < (int)lorrext_plain_h.size() && lorrext_plain_h[ai] && !lns.heuristics[lns.tasks.at(ai)].empty())\n        return get_heuristic(lns.heuristics[lns.tasks.at(ai)], lns.env, target, &(lns.neighbors));", "plain heuristic"),
        ], "LORR_EXT_PLAN_AHEAD")
        done.append("plan-ahead")
    if pibt_reserve is not None:
        # The 2026 planner reserves MIN_PIBT_TIME ms for PIBT and gives guidance the rest of the
        # window; 100 ms leaves short windows no guidance at all (docs/research/E1_2026_execution.md).
        patch_file(kit / "default_planner" / "const.h", [
            (r"const\s+int\s+MIN_PIBT_TIME\s*=\s*\d+\s*;", f"const int MIN_PIBT_TIME = {pibt_reserve}; /* LoRR-Ext */", "pibt reserve"),
        ], "LORR_EXT_PIBT_RESERVE")
        done.append(f"pibt reserve {pibt_reserve} ms")
    if kitty_resync:
        # Kitty_Knight advances its plan only if every robot reached its planned state and
        # exit(-1)s when some moved and others didn't (any delay or breakdown). Instead: if
        # nobody moved, retry the step; otherwise drop the plans and restart from the actual
        # states (LaCAM2 rebuilds the window). docs/research/B1_breakdowns.md
        patch_file(kit / "wppl_planner" / "src" / "LNS" / "LNSSolver.cpp", [
            (r"if\s*\(\s*match\s*\)\s*\{\s*\+\+executed_step\s*;\s*\}",
             """if (match){
            ++executed_step;
        } else { /* LoRR-Ext resync */
            bool stayed = true;
            for (int i=0;i<execution_paths.size();++i)
                if (execution_paths[i][executed_step].location!=env.curr_states[i].location || execution_paths[i][executed_step].orientation!=env.curr_states[i].orientation)
                    stayed = false;
            if (!stayed) {
                for (int i=0;i<execution_paths.size();++i) {
                    execution_paths[i].clear(); execution_paths[i].push_back(env.curr_states[i]);
                    planning_paths[i].clear(); planning_paths[i].push_back(env.curr_states[i]);
                }
                executed_step=0;
            }
        }""", "resync"),
        ], "LORR_EXT_KITTY_RESYNC")
        done.append("kitty resync")
    if fault_aware:
        # Tests the B1 hypothesis (docs/research/B1_breakdowns.md): breakdowns cost far more
        # than the frozen time because the planner does not know a robot is frozen. Every tick
        # PIBT plans the frozen robot as moving and plans robots behind it into the cell it
        # "vacates"; the simulator's local safety stop then cancels those moves too (the
        # cascade waits, 2.7-3.7x the frozen time). This variant tells the planner which robots
        # are stopped (broken down, a slow robot's off tick, a pause), the way a real fleet
        # manager knows a robot reported a fault, and plans them as staying put, so PIBT routes
        # the others around them. If the hypothesis holds, cascade waits and most of the loss
        # disappear.
        patch_file(kit / "default_planner" / "planner.cpp", [
            (r'(#include\s+"planner.h")', r'\1\n#include "lorr_ext.h"', "include"),
            # Inserted just before PIBT runs, after the planner has rebuilt this tick's
            # `decision` table (cell -> robot that claims it) and `decided` (each robot's
            # committed target cell, NOT_DONE while it is still turning towards it).
            (r"(\n(\s*)// compute the targeted next location for each agent using PIBT)",
             r"""
\2// LoRR-Ext fault-aware: a frozen robot will not move this tick, so plan it as staying put.
\2// A robot keeps a committed target cell (decided NOT_DONE) over several ticks while it
\2// turns, and one whose move was blocked keeps it too, so a broken robot usually has one.
\2// Robots behind it may in turn be committed to the cell it was going to vacate. Pinning
\2// the frozen robot alone left those commitments pointing into its cell, and two robots
\2// later swapped cells (an edge conflict: the 2024 kit then stops every robot, here
\2// forever). So we cancel, transitively, every commitment into a cell whose robot now
\2// stays, and let PIBT replan those robots together with the free ones.
\2std::vector<char> lorrext_stays(env->num_of_agents, 0);
\2for (int i : ids)
\2    lorrext_stays[i] = LorrExt::frozen(i, env->curr_timestep);
\2for (bool changed = true; changed;) {
\2    changed = false;
\2    for (int j : ids) {
\2        if (lorrext_stays[j] || decided[j].state != DONE::NOT_DONE) continue;
\2        int k = prev_decision.at(decided[j].loc); // robot standing in j's target cell
\2        if (k < 0 || !lorrext_stays[k]) continue;
\2        // cancel j's commitment: PIBT replans robots whose next_states is unset
\2        if (decision.at(decided[j].loc) == j) decision.at(decided[j].loc) = -1;
\2        decided[j] = DCR({prev_states[j].location, DONE::DONE});
\2        next_states[j] = State();
\2        // j may now have to stay too, so robots committed into j's cell are released next
\2        lorrext_stays[j] = 1;
\2        changed = true;
\2    }
\2}
\2for (int i : ids) {
\2    if (!LorrExt::frozen(i, env->curr_timestep)) continue;
\2    // drop the frozen robot's own commitment; it resumes planning once it is unfrozen
\2    if (decided[i].state == DONE::NOT_DONE && decision.at(decided[i].loc) == i) decision.at(decided[i].loc) = -1;
\2    decided[i] = DCR({prev_states[i].location, DONE::DONE});
\2    // fix its next cell to its current cell: PIBT skips robots whose next_states is set
\2    // and treats the claimed cell as occupied, so nobody is planned into it
\2    next_states[i] = State(prev_states[i].location, -1, -1);
\2    decision.at(prev_states[i].location) = i;
\2}\1""", "fault-aware"),
        ], "LORR_EXT_FAULT_AWARE")
        done.append("fault-aware")
    return done


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kit", type=Path)
    ap.add_argument("--kitty-guard", action="store_true")
    ap.add_argument("--stale-task-guard", action="store_true")
    ap.add_argument("--nms-arrival-fix", action="store_true")
    ap.add_argument("--plan-ahead", action="store_true", help="2026 default planner: next errand inside the rollout")
    ap.add_argument("--pibt-reserve", type=int, default=None, help="2026 default planner: MIN_PIBT_TIME in ms")
    ap.add_argument("--kitty-resync", action="store_true", help="Kitty_Knight: resync instead of exit on plan mismatch")
    ap.add_argument("--fault-aware", action="store_true", help="2024 default planner: plan stopped robots as staying")
    a = ap.parse_args()
    print("patched:", ", ".join(patch(a.kit, a.kitty_guard, a.stale_task_guard, a.nms_arrival_fix, a.plan_ahead,
                                      a.pibt_reserve, a.kitty_resync, a.fault_aware)))


if __name__ == "__main__":
    main()
