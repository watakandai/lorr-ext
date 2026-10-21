// LoRR-Ext baseline schedulers for the 2026 Start-Kit.
//
// Selected with "lorrExt": {"scheduler": NAME} in the input JSON:
//   default : the kit's greedy scheduler (each free robot takes its
//             shortest task), untouched
//   fifo    : oldest open task first, each given to the nearest free robot
//   slack   : least slack first, slack = deadline - (age + errand path +
//             safe service), where safe service is a quantile (default 95th
//             percentile) of the *known* service-time distributions
//   stage   : fifo, plus idle robots are sent to (parked on) the start
//             errand of recently revealed tasks
//   stage_near : like stage, but parks on a free cell next to that errand
//             instead of on it
//
// All baselines commit on assignment (no re-assignment of unopened tasks),
// like the default, so differences come from the order alone.

#include <algorithm>
#include <climits>
#include <deque>
#include <unordered_set>

#include "heuristics.h"
#include "lorr_ext.h"
#include "scheduler.h"

namespace LorrExtSched
{

static std::deque<int> recent_starts; // start errands of recently revealed tasks
static std::vector<int> idle_goal_of;  // agent -> parking cell (-1 = none)

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

// "safe" remaining service: the planQuantile (default 95th percentile) of
// each remaining errand's known duration distribution
static int safe_service(const Task &t)
{
    int s = 0;
    for (size_t j = std::max(0, t.idx_next_loc); j < t.locations.size(); j++)
        s += LorrExt::service_quantile(int(j), LorrExt::cfg().serviceQuantile);
    return s;
}

static int free_neighbour(SharedEnvironment *env, int loc, const std::unordered_set<int> &taken)
{
    int r = loc / env->cols, c = loc % env->cols;
    const int dr[4] = {0, 1, 0, -1}, dc[4] = {1, 0, -1, 0};
    for (int k = 0; k < 4; k++)
    {
        int nr = r + dr[k], nc = c + dc[k];
        if (nr < 0 || nc < 0 || nr >= env->rows || nc >= env->cols)
            continue;
        int n = nr * env->cols + nc;
        if (env->map[n] == 0 && !taken.count(n))
            return n;
    }
    return -1;
}

// Returns false when the default scheduler should run instead.
bool plan(int time_limit, std::vector<int> &proposed_schedule, SharedEnvironment *env)
{
    const std::string &name = LorrExt::cfg().scheduler;
    if (!LorrExt::cfg().enabled || name == "default")
        return false;

    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    const int now = env->curr_timestep;
    const int D = LorrExt::cfg().deadline;

    for (int t : env->new_tasks)
    {
        recent_starts.push_back(env->task_pool[t].locations.front());
        if ((int)recent_starts.size() > env->num_of_agents)
            recent_starts.pop_front();
    }

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
        long long key = t.t_revealed; // fifo / stage
        if (name == "slack")
        {
            // travel of the nearest free robot is added below per-robot;
            // here the robot-independent part keeps sorting cheap
            int age = now - t.t_revealed;
            key = (long long)D - age - path_len(env, t, t.locations.front()) - safe_service(t);
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

    // staging goals for robots that stay idle
    idle_goal_of.assign(env->num_of_agents, -1);
    if (name == "stage" || name == "stage_near")
    {
        std::unordered_set<int> taken;
        for (int i = 0; i < env->num_of_agents; i++)
            if (proposed_schedule[i] != -1)
                taken.insert(env->curr_states[i].location);
        for (auto it = recent_starts.rbegin(); it != recent_starts.rend(); ++it)
        {
            int cell = *it;
            if (name == "stage_near")
                cell = free_neighbour(env, cell, taken);
            if (cell < 0 || taken.count(cell))
                continue;
            int best = -1, best_d = INT_MAX;
            for (int i : free_agents)
            {
                if (proposed_schedule[i] != -1 || idle_goal_of[i] != -1)
                    continue;
                int d = DefaultPlanner::get_h(env, env->curr_states[i].location, cell);
                if (d < best_d)
                {
                    best_d = d;
                    best = i;
                }
            }
            if (best < 0)
                break;
            idle_goal_of[best] = cell;
            taken.insert(cell);
        }
    }
    return true;
}

int idle_goal(int agent)
{
    if (agent < 0 || agent >= (int)idle_goal_of.size())
        return -1;
    return idle_goal_of[agent];
}

} // namespace LorrExtSched
