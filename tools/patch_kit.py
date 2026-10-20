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


def sub_once(text: str, pattern: str, repl, path: Path, what: str) -> str:
    new, n = re.subn(pattern, repl, text, count=1, flags=re.S)
    if n != 1:
        sys.exit(f"patch_kit: anchor for {what} not found in {path}")
    return new


def patch_file(path: Path, edits) -> None:
    text = path.read_text()
    if MARK in text:
        return
    for pattern, repl, what in edits:
        text = sub_once(text, pattern, repl, path, what)
    path.write_text(f"// {MARK}\n" + text)


def patch(kit: Path, kitty_guard: bool = False, stale_guard: bool = False) -> list[str]:
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

    # 2026 kit only: baseline schedulers + idle-robot staging goals
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
        patch_file(kit / "src" / "Entry.cpp", [
            (r"(#include\s+\"Entry.h\")", r"\1\nnamespace LorrExtSched { int idle_goal(int); }", "include"),
            (r"if\s*\(\s*t_id\s*==\s*-1\s*\)\s*continue\s*;",
             "if (t_id == -1)\n        {\n            int g = LorrExtSched::idle_goal(i);\n"
             "            if (g >= 0) env->goal_locations[i].push_back({g, env->curr_timestep});\n"
             "            continue;\n        }", "idle-goal hook"),
        ])
        done.append("2026 baseline schedulers")

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
    return done


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kit", type=Path)
    ap.add_argument("--kitty-guard", action="store_true")
    ap.add_argument("--stale-task-guard", action="store_true")
    a = ap.parse_args()
    print("patched:", ", ".join(patch(a.kit, a.kitty_guard, a.stale_task_guard)))


if __name__ == "__main__":
    main()
