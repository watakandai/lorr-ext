// LoRR-Ext: known arrival processes, hidden interval service times and
// per-task latency records for the League of Robot Runners Start-Kit.
//
// Header-only so it builds unchanged inside the 2026 kit and the 2024-season
// kits used by archived entries (some of which list sources by hand in CMake).
// It is wired in by tools/patch_kit.py through four hooks:
//   driver.cpp         LorrExt::configure(data, team_size)
//   TaskManager.cpp    LorrExt::reveal(...)        start of reveal_tasks
//                      LorrExt::stop_reached(...)  replaces the "agent is on its
//                                                  next errand" test
//   CompetitionSystem  LorrExt::save(...)          inside saveResults
// With no "lorrExt" object in the input JSON every hook is a no-op and the kit
// behaves exactly as released.
//
// Schedulers may read the *known* model through LorrExt::cfg() and
// LorrExt::service_interval(); the realised service time stays hidden.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <list>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"

struct LorrExtConfig
{
    bool enabled = false;

    // ---- arrivals ------------------------------------------------------
    // "pool"    : original kit behaviour (keep numTasksReveal*teamSize open)
    // "poisson" : N_t ~ Poisson(rate) new tasks per tick
    // "mmpp"    : two-state Markov-modulated Poisson. A "burst" state with rate
    //             rate*burstFactor occupies burstFraction of the time on
    //             average (mean sojourn burstLength ticks); the calm-state rate
    //             is chosen so the long-run mean is still `rate`.
    // "hawkes"  : self-exciting; intensity mu + sum alpha*exp(-beta*(t-t_i)),
    //             branching ratio n = alpha/beta, mu = rate*(1-n).
    std::string arrival = "pool";
    double rate = 0.5;
    double burstFactor = 4.0;
    double burstFraction = 0.2;
    double burstLength = 50.0;
    double branching = 0.6;
    double decay = 0.05; // beta
    uint64_t arrivalSeed = 0;

    // ---- service times -------------------------------------------------
    // Stop j of every task needs the robot to be on the errand cell for
    // d ~ U{lo_j, hi_j} ticks (cumulative; being pushed off pauses the clock),
    // with [lo_j, hi_j] = serviceIntervals[j % size]. Empty => instantaneous.
    std::vector<std::pair<int, int>> serviceIntervals;
    uint64_t serviceSeed = 0;

    // ---- metrics / baselines --------------------------------------------
    int deadline = 300;              // informational; analysis can re-threshold
    std::string scheduler = "default"; // 2026-kit baseline selector
};

struct LorrExt
{
    // ------------------------------------------------------------- state
    static LorrExtConfig &cfg()
    {
        static LorrExtConfig c;
        return c;
    }

    struct State
    {
        std::mt19937_64 rng;
        int last_t = -1;       // last tick whose arrivals were generated
        bool burst = false;    // mmpp state
        double excitation = 0; // hawkes sum alpha*exp(-beta*(t-t_i))
        // dwell bookkeeping: (task_id, stop_idx) -> (serving agent, ticks served)
        std::unordered_map<int64_t, std::pair<int, int>> served;
        std::unordered_map<int, int> last_seen; // agent -> last check tick
        std::vector<std::pair<int, int>> arrivals_per_tick; // (t, n) with n>0
    };
    static State &st()
    {
        static State s;
        return s;
    }

    // ------------------------------------------------------------- config
    static void configure(const nlohmann::json &data, int /*team_size*/)
    {
        if (!data.contains("lorrExt"))
            return;
        const auto &j = data["lorrExt"];
        auto &c = cfg();
        c.enabled = true;
        auto get = [&](const nlohmann::json &o, const char *k, auto def) {
            return o.contains(k) ? o[k].get<decltype(def)>() : def;
        };
        if (j.contains("arrival"))
        {
            const auto &a = j["arrival"];
            c.arrival = get(a, "model", c.arrival);
            c.rate = get(a, "rate", c.rate);
            c.burstFactor = get(a, "burstFactor", c.burstFactor);
            c.burstFraction = get(a, "burstFraction", c.burstFraction);
            c.burstLength = get(a, "burstLength", c.burstLength);
            c.branching = get(a, "branching", c.branching);
            c.decay = get(a, "decay", c.decay);
            c.arrivalSeed = get(a, "seed", c.arrivalSeed);
        }
        if (j.contains("service"))
        {
            const auto &s = j["service"];
            if (s.contains("intervals"))
                for (const auto &iv : s["intervals"])
                    c.serviceIntervals.emplace_back(iv[0].get<int>(), iv[1].get<int>());
            c.serviceSeed = get(s, "seed", c.serviceSeed);
        }
        c.deadline = get(j, "deadline", c.deadline);
        c.scheduler = get(j, "scheduler", c.scheduler);
        st().rng.seed(c.arrivalSeed * 0x9E3779B97F4A7C15ULL + 12345);
    }

    static bool custom_arrivals() { return cfg().enabled && cfg().arrival != "pool"; }

    // Known service interval of stop `stop_idx` (what a scheduler may use).
    static std::pair<int, int> service_interval(int stop_idx)
    {
        const auto &iv = cfg().serviceIntervals;
        if (!cfg().enabled || iv.empty())
            return {0, 0};
        return iv[stop_idx % iv.size()];
    }

    // Hidden realised service time; a hash of (seed, task, stop) so it is the
    // same for every solver regardless of call order.
    static int service_time(int task_id, int stop_idx)
    {
        auto iv = service_interval(stop_idx);
        if (iv.second <= 0)
            return 0;
        uint64_t x = cfg().serviceSeed * 0x9E3779B97F4A7C15ULL ^ (uint64_t(task_id) << 20) ^ uint64_t(stop_idx);
        x ^= x >> 33; x *= 0xff51afd7ed558ccdULL; x ^= x >> 33; x *= 0xc4ceb9fe1a85ec53ULL; x ^= x >> 33;
        return iv.first + int(x % uint64_t(iv.second - iv.first + 1));
    }

    // Number of arrivals in tick t (advances the process by one tick).
    static int draw_arrivals(int t)
    {
        auto &c = cfg();
        auto &s = st();
        double lambda = c.rate;
        if (c.arrival == "mmpp")
        {
            // switching probabilities giving mean sojourns burstLength (burst)
            // and burstLength*(1-f)/f (calm)
            double f = std::clamp(c.burstFraction, 1e-6, 1 - 1e-6);
            double p_exit_burst = 1.0 / std::max(1.0, c.burstLength);
            double p_enter_burst = p_exit_burst * f / (1 - f);
            std::uniform_real_distribution<double> u(0, 1);
            if (t == 0)
                s.burst = u(s.rng) < f;
            else if (s.burst ? u(s.rng) < p_exit_burst : u(s.rng) < p_enter_burst)
                s.burst = !s.burst;
            double hi = c.rate * c.burstFactor;
            double lo = std::max(0.0, (c.rate - f * hi) / (1 - f));
            lambda = s.burst ? hi : lo;
        }
        else if (c.arrival == "hawkes")
        {
            double n = std::clamp(c.branching, 0.0, 0.99);
            double mu = c.rate * (1 - n);
            lambda = mu + s.excitation;
        }
        std::poisson_distribution<int> pois(std::max(lambda, 1e-12));
        int k = pois(s.rng);
        if (c.arrival == "hawkes")
        {
            double alpha = c.branching * c.decay; // integral of kernel = branching
            s.excitation = s.excitation * std::exp(-c.decay) + alpha * k;
        }
        return k;
    }

    // Hook at the start of TaskManager::reveal_tasks. Returns true when it
    // handled the reveal (caller then returns immediately).
    template <class TM>
    static bool reveal(TM &tm, int timestep)
    {
        if (!custom_arrivals())
            return false;
        auto &s = st();
        for (int t = s.last_t + 1; t <= timestep; t++)
        {
            int k = draw_arrivals(t);
            if (k > 0)
                s.arrivals_per_tick.emplace_back(t, k);
            for (int i = 0; i < k; i++)
            {
                int idx = tm.task_id % tm.tasks.size();
                std::list<int> locs = tm.tasks[idx];
                auto *task = new typename std::remove_pointer<typename decltype(tm.all_tasks)::value_type>::type(tm.task_id, locs, timestep);
                tm.ongoing_tasks[task->task_id] = task;
                tm.all_tasks.push_back(task);
                tm.new_tasks.push_back(task->task_id);
                tm.task_id++;
            }
        }
        s.last_t = std::max(s.last_t, timestep);
        return true;
    }

    // Replaces `states[k].location == ongoing_tasks[current_assignment[k]]->get_next_loc()`.
    // True once agent k has been on its next errand for the hidden service time.
    template <class TM>
    static bool stop_reached(TM &tm, int k, int location)
    {
        auto *task = tm.ongoing_tasks[tm.current_assignment[k]];
        if (location != task->get_next_loc())
        {
            st().last_seen.erase(k);
            return false;
        }
        if (!cfg().enabled || cfg().serviceIntervals.empty())
            return true;
        int t = tm.curr_timestep;
        auto &s = st();
        int64_t key = (int64_t(task->task_id) << 8) | task->idx_next_loc;
        auto ls = s.last_seen.find(k);
        // count ticks actually spent on the cell since the previous check
        int inc = (ls == s.last_seen.end()) ? 1 : std::max(1, t - ls->second);
        s.last_seen[k] = t;
        auto &sv = s.served[key];
        if (sv.first != k)
            sv = {k, 0}; // a different robot took over this stop: the clock restarts
        sv.second += inc;
        if (sv.second >= service_time(task->task_id, task->idx_next_loc))
        {
            s.served.erase(key);
            s.last_seen.erase(k);
            return true;
        }
        return false;
    }

    // Writes compact per-task records into the output JSON.
    template <class TM>
    static void save(const TM &tm, nlohmann::ordered_json &js, int simulation_time)
    {
        const auto &c = cfg();
        nlohmann::ordered_json ext;
        ext["enabled"] = c.enabled;
        ext["arrival"] = c.enabled ? c.arrival : "pool";
        ext["rate"] = c.rate;
        ext["deadline"] = c.deadline;
        ext["scheduler"] = c.scheduler;
        ext["simulationTime"] = simulation_time;
        nlohmann::ordered_json iv = nlohmann::ordered_json::array();
        for (auto &p : c.serviceIntervals)
            iv.push_back({p.first, p.second});
        ext["serviceIntervals"] = iv;
        // [task_id, t_revealed, t_completed (-1 = open), agent, n_stops]
        nlohmann::ordered_json recs = nlohmann::ordered_json::array();
        for (auto *t : tm.all_tasks)
            recs.push_back({t->task_id, t->t_revealed, t->t_completed, t->agent_assigned, (int)t->locations.size()});
        ext["tasks"] = recs;
        js["lorrExt"] = ext;
    }
    template <class TM>
    static void save(const TM &tm, nlohmann::json &js, int simulation_time)
    {
        nlohmann::ordered_json o;
        save(tm, o, simulation_time);
        js["lorrExt"] = nlohmann::json::parse(o["lorrExt"].dump());
    }
};
