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
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"

// Known distribution of one errand's service (dwell) time, in ticks.
//   uniform    : integer U{lo, hi} (the "interval" setting)
//   exponential: mean                      (cv is 1 by definition)
//   lognormal  : mean, cv                  (right-skewed, never negative)
//   gamma      : mean, cv                  (shape 1/cv^2)
//   normal     : mean, cv, truncated at min
//   fixed      : mean                      (deterministic)
// Draws are rounded to whole ticks and clamped to [min, max].
struct ServiceDist
{
    std::string type = "uniform";
    double lo = 0, hi = 0;       // uniform
    double mean = 0, cv = 0;     // others
    double min = 0, max = 1e9;
    std::vector<int> qtable;     // empirical quantiles q = 0, 0.001, ..., 1

    template <class RNG>
    double draw_raw(RNG &g) const
    {
        if (type == "uniform")
            return std::uniform_int_distribution<int>(int(lo), int(hi))(g);
        if (type == "fixed")
            return mean;
        if (type == "exponential")
            return std::exponential_distribution<double>(1.0 / std::max(mean, 1e-9))(g);
        if (type == "lognormal")
        {
            double s2 = std::log(1 + cv * cv);
            return std::lognormal_distribution<double>(std::log(std::max(mean, 1e-9)) - s2 / 2, std::sqrt(s2))(g);
        }
        if (type == "gamma")
        {
            double k = 1.0 / std::max(cv * cv, 1e-9);
            return std::gamma_distribution<double>(k, mean / k)(g);
        }
        if (type == "normal")
            return std::normal_distribution<double>(mean, mean * cv)(g);
        throw std::invalid_argument("unknown service distribution: " + type);
    }
    template <class RNG>
    int draw(RNG &g) const
    {
        double x = std::round(draw_raw(g));
        return int(std::clamp(x, min, max));
    }
    // empirical quantile table from a fixed-seed sample, so every distribution
    // gets quantiles without special functions
    void build_quantiles()
    {
        std::mt19937_64 g(424242);
        std::vector<int> xs(20000);
        for (auto &x : xs)
            x = draw(g);
        std::sort(xs.begin(), xs.end());
        qtable.resize(1001);
        for (int i = 0; i <= 1000; i++)
            qtable[i] = xs[std::min<size_t>(xs.size() - 1, size_t(i / 1000.0 * (xs.size() - 1)))];
    }
    int quantile(double q) const
    {
        if (qtable.empty())
            return 0;
        return qtable[std::clamp(int(std::lround(q * 1000)), 0, 1000)];
    }
    double expected() const
    {
        double m = 0;
        for (int i = 0; i <= 1000; i++)
            m += qtable[i];
        return qtable.empty() ? 0 : m / qtable.size();
    }
};

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
    // Errand j of every task needs the robot to be on the errand cell for d
    // ticks (cumulative; being pushed off pauses the clock), with
    // d ~ service[j % size]. Empty => instantaneous (the original kit).
    std::vector<ServiceDist> service;
    uint64_t serviceSeed = 0;
    double serviceQuantile = 0.95;   // the "safe" duration baselines plan with

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
            // "intervals": [[lo, hi], ...]  (shorthand for uniform)
            if (s.contains("intervals"))
                for (const auto &iv : s["intervals"])
                {
                    ServiceDist d;
                    d.lo = iv[0].get<int>(); d.hi = iv[1].get<int>();
                    d.min = d.lo; d.max = d.hi;
                    c.service.push_back(d);
                }
            // "stops": [{"dist": "lognormal", "mean": 20, "cv": 0.5, "min": 1, "max": 200}, ...]
            if (s.contains("stops"))
                for (const auto &o : s["stops"])
                {
                    ServiceDist d;
                    d.type = get(o, "dist", std::string("fixed"));
                    d.lo = get(o, "lo", 0.0); d.hi = get(o, "hi", 0.0);
                    d.mean = get(o, "mean", 0.0); d.cv = get(o, "cv", 0.0);
                    d.min = get(o, "min", d.type == "uniform" ? d.lo : 0.0);
                    d.max = get(o, "max", d.type == "uniform" ? d.hi : 1e9);
                    c.service.push_back(d);
                }
            for (auto &d : c.service)
                d.build_quantiles();
            c.serviceSeed = get(s, "seed", c.serviceSeed);
            c.serviceQuantile = get(s, "planQuantile", c.serviceQuantile);
        }
        c.deadline = get(j, "deadline", c.deadline);
        c.scheduler = get(j, "scheduler", c.scheduler);
        st().rng.seed(c.arrivalSeed * 0x9E3779B97F4A7C15ULL + 12345);
    }

    static bool custom_arrivals() { return cfg().enabled && cfg().arrival != "pool"; }

    // ---- what a scheduler may know: the distribution, not the draw ----
    static const ServiceDist *service_dist(int stop_idx)
    {
        const auto &sv = cfg().service;
        if (!cfg().enabled || sv.empty())
            return nullptr;
        return &sv[stop_idx % sv.size()];
    }
    static double service_mean(int stop_idx)
    {
        auto *d = service_dist(stop_idx);
        return d ? d->expected() : 0.0;
    }
    static int service_quantile(int stop_idx, double q)
    {
        auto *d = service_dist(stop_idx);
        return d ? d->quantile(q) : 0;
    }
    // kept for the interval setting: (lo, hi) = (q0, q1)
    static std::pair<int, int> service_interval(int stop_idx)
    {
        return {service_quantile(stop_idx, 0.0), service_quantile(stop_idx, 1.0)};
    }

    // Hidden realised service time: drawn from a generator seeded by a hash of
    // (seed, task, stop), so it is the same for every solver and call order.
    static int service_time(int task_id, int stop_idx)
    {
        auto *d = service_dist(stop_idx);
        if (!d)
            return 0;
        uint64_t x = cfg().serviceSeed * 0x9E3779B97F4A7C15ULL ^ (uint64_t(task_id) << 20) ^ uint64_t(stop_idx);
        x ^= x >> 33; x *= 0xff51afd7ed558ccdULL; x ^= x >> 33; x *= 0xc4ceb9fe1a85ec53ULL; x ^= x >> 33;
        std::mt19937_64 g(x);
        return d->draw(g);
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
        if (!cfg().enabled || cfg().service.empty())
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
        nlohmann::ordered_json sv = nlohmann::ordered_json::array();
        for (auto &d : c.service)
            sv.push_back({{"dist", d.type}, {"lo", d.lo}, {"hi", d.hi}, {"mean", d.mean}, {"cv", d.cv},
                          {"min", d.min}, {"max", std::min(d.max, 1e6)},
                          {"q05", d.quantile(0.05)}, {"q50", d.quantile(0.5)}, {"q95", d.quantile(0.95)}});
        ext["service"] = sv;
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
