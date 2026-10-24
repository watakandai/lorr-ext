// LoRR-Ext baseline schedulers for the 2026 Start-Kit.
//
// Selected with "lorrExt": {"scheduler": NAME} in the input JSON:
//   default : the kit's greedy scheduler (each free robot takes its
//             shortest task), untouched
//   fifo    : oldest open task first, each given to the nearest free robot
//   slack   : least slack first, slack = deadline - (age + errand path +
//             safe service), where safe service is a quantile (default 95th
//             percentile) of the *known* service-time distribution of each
//             remaining errand's cell, and deadline is the task's own
//             (LorrExt::deadline_of: its deadline class, if classes are set)
//
// All baselines commit on assignment (no re-assignment of unopened tasks),
// like the default, so differences come from the order alone.

#include <algorithm>
#include <climits>
#include <unordered_set>

#include "heuristics.h"
#include "lorr_ext.h"
#include "scheduler.h"

namespace LorrExtSched
{

static int path_len(SharedEnvironment *env, const Task &t, int from)
{
    int d = 0, c = from;
    for (size_t j = std::max(0, t.idx_next_loc); j < t.locations.size(); j++)
    {
        d += DefaultPlanner::get_h(env, c, t.locations[j]);
        c = t.locations[j];
    }
    return d;
}

// "safe" remaining service: the planQuantile (default 95th percentile) of the
// known service-time distribution at each remaining errand's cell
static int safe_service(const Task &t)
{
    int s = 0;
    for (size_t j = std::max(0, t.idx_next_loc); j < t.locations.size(); j++)
        s += LorrExt::service_quantile_at(t.locations[j], LorrExt::cfg().serviceQuantile);
    return s;
}

// Returns false when the default scheduler should run instead.
bool plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
{
    const std::string &name = LorrExt::cfg().scheduler;
    if (!LorrExt::cfg().enabled || name == "default")
        return false;

    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    const int now = env->curr_timestep;

    std::unordered_set<int> assigned;
    std::vector<int> free_agents;
    for (int i = 0; i < env->num_of_agents; i++)
    {
        int t = proposed_schedule[i];
        if (t != -1 && env->task_pool.count(t))
            assigned.insert(t);
        else
        {
            proposed_schedule[i] = -1;
            free_agents.push_back(i);
        }
    }

    // candidate tasks with a priority key (smaller = more urgent)
    std::vector<std::pair<long long, int>> order;
    for (auto &kv : env->task_pool)
    {
        if (assigned.count(kv.first))
            continue;
        const Task &t = kv.second;
        long long key = t.t_revealed; // fifo
        if (name == "slack")
        {
            int age = now - t.t_revealed;
            key = (long long)LorrExt::deadline_of(t.task_id) - age - path_len(env, t, t.locations.front()) - safe_service(t);
        }
        order.emplace_back(key, kv.first);
    }
    std::sort(order.begin(), order.end());
    if (order.size() > 256)
        order.resize(256); // bound per-call work; the rest wait one call

    std::vector<char> used(env->num_of_agents, 0);
    int n_free = (int)free_agents.size();
    for (auto &kt : order)
    {
        if (n_free == 0 || std::chrono::steady_clock::now() > endtime)
            break;
        const Task &t = env->task_pool[kt.second];
        int best = -1, best_d = INT_MAX;
        for (int i : free_agents)
        {
            if (used[i])
                continue;
            int d = DefaultPlanner::get_h(env, env->curr_states[i].location, t.locations.front());
            if (d < best_d)
            {
                best_d = d;
                best = i;
            }
        }
        if (best < 0)
            break;
        proposed_schedule[best] = kt.second;
        used[best] = 1;
        n_free--;
    }
    return true;
}

} // namespace LorrExtSched
