// Task-allocation experiments:
//   static   Hungarian vs. auction (Gauss-Seidel, Jacobi) vs. greedy on random
//            Euclidean instances: equal counts, more tasks than robots, more
//            robots than tasks; cost gap, runtime, rounds and messages
//   dynamic  tasks arrive as a Poisson process; dynamic reallocation with
//            different switching penalties (completion time, reassignments)
//   multi    SSI auctions (MiniSum / MiniMax) for several tasks per robot
//
// mrc-modules: allocation io

#include "demo_common.hpp"
#include "experiment_common.hpp"

#include "multirobotcore/allocation/auction.hpp"
#include "multirobotcore/allocation/dynamic.hpp"
#include "multirobotcore/allocation/hungarian.hpp"
#include "multirobotcore/allocation/sequential_auction.hpp"
#include "multirobotcore/math/numerics.hpp"
#include "multirobotcore/math/random.hpp"

#include <limits>
#include <random>
#include <string>
#include <vector>

namespace mrc = multirobotcore;
namespace al = multirobotcore::allocation;
namespace io = multirobotcore::io;
using mrc::Vec2;
using mrc_apps::Json;
using mrc_apps::Sample;

namespace {

std::vector<al::Robot> random_robots(std::size_t n, double side, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> u(0.0, side);
  std::vector<al::Robot> out(n);
  for (std::size_t i = 0; i < n; ++i) {
    out[i].id = i;
    out[i].position = mrc::math::sample_vec2(rng, u);
  }
  return out;
}

std::vector<al::Task> random_tasks(std::size_t m, double side, std::mt19937_64& rng) {
  std::uniform_real_distribution<double> u(0.0, side);
  std::vector<al::Task> out(m);
  for (std::size_t j = 0; j < m; ++j) {
    out[j].id = j;
    out[j].position = mrc::math::sample_vec2(rng, u);
  }
  return out;
}

double greedy_cost(const al::CostMatrix& c) {
  const auto n = static_cast<std::size_t>(c.rows());
  const auto m = static_cast<std::size_t>(c.cols());
  std::vector<bool> ru(n, false);
  std::vector<bool> tu(m, false);
  double total = 0.0;
  for (std::size_t k = 0; k < std::min(n, m); ++k) {
    double best = std::numeric_limits<double>::infinity();
    std::size_t bi = 0;
    std::size_t bj = 0;
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = 0; j < m; ++j) {
        const double v = c(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j));
        if (!ru[i] && !tu[j] && v < best) {
          best = v;
          bi = i;
          bj = j;
        }
      }
    }
    ru[bi] = tu[bj] = true;
    total += best;
  }
  return total;
}

struct DynamicOutcome {
  double mean_completion{0.0};
  double max_completion{0.0};
  std::size_t reassignments{0};
  std::size_t completed{0};
};

/// Kinematic dynamic-allocation run: robots move straight at unit speed.
DynamicOutcome run_dynamic(std::size_t robots_n, double rate, double horizon, double penalty,
                           std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  const double side = 20.0;
  std::uniform_real_distribution<double> u(0.0, side);
  std::exponential_distribution<double> gap(rate);
  al::DynamicAllocatorConfig cfg;
  cfg.switching_penalty = penalty;
  cfg.commit_distance = 1.0;
  al::DynamicTaskAllocator allocator(cfg);
  std::vector<Vec2> pos(robots_n);
  for (auto& p : pos) {
    p = mrc::math::sample_vec2(rng, u);
  }
  std::vector<double> arrival_time;
  std::vector<std::optional<std::size_t>> current(robots_n);
  double next_arrival = gap(rng);
  const double dt = 0.1;
  DynamicOutcome out;
  double sum = 0.0;
  bool dirty = false;
  for (double t = 0.0; t < horizon || !allocator.open_tasks().empty(); t += dt) {
    if (t > 4.0 * horizon) {
      break;
    }
    while (t < horizon && t >= next_arrival) {
      al::Task task;
      task.position = mrc::math::sample_vec2(rng, u);
      allocator.add_task(task);
      arrival_time.push_back(next_arrival);
      next_arrival += gap(rng);
      dirty = true;
    }
    if (dirty) {
      std::vector<al::Robot> rs(robots_n);
      for (std::size_t i = 0; i < robots_n; ++i) {
        rs[i].id = i;
        rs[i].position = pos[i];
      }
      const auto upd = allocator.update(rs, current);
      out.reassignments += upd.reassigned;
      current = upd.robot_task;
      dirty = false;
    }
    for (std::size_t i = 0; i < robots_n; ++i) {
      if (!current[i]) {
        continue;
      }
      const auto task = allocator.find_task(*current[i]);
      if (!task) {
        current[i].reset();
        dirty = true;
        continue;
      }
      const Vec2 d = task->position - pos[i];
      if (d.norm() <= dt) {
        pos[i] = task->position;
        const double latency = t - arrival_time[*current[i]];
        sum += latency;
        out.max_completion = std::max(out.max_completion, latency);
        ++out.completed;
        allocator.complete_task(*current[i]);
        current[i].reset();
        dirty = true;
      } else {
        pos[i] += dt * d.normalized();
      }
    }
  }
  out.mean_completion = out.completed > 0 ? sum / static_cast<double>(out.completed) : 0.0;
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args = mrc_apps::parse_args(argc, argv, "allocation_benchmark",
                                           "configs/allocation/benchmark.json");
    const Json cfg = mrc_apps::load_config(args);
    const auto seeds = cfg.value("seeds", std::size_t{5});

    Json static_rows = Json::array();
    for (const auto& shape : cfg.at("shapes")) {
      const std::string name = shape.at("name").get<std::string>();
      const double task_ratio = shape.at("tasks_per_robot").get<double>();
      for (const auto& n_json : cfg.at("team_sizes")) {
        const auto n = n_json.get<std::size_t>();
        const auto m =
            std::max<std::size_t>(1, static_cast<std::size_t>(task_ratio * static_cast<double>(n)));
        Sample h_cost, h_time, g_gap, gs_gap, gs_time, gs_msgs, ja_gap, ja_time, ja_rounds, ja_msgs;
        for (std::size_t s = 0; s < seeds; ++s) {
          std::mt19937_64 rng(42 + s);
          const auto rs = random_robots(n, 50.0, rng);
          const auto ts = random_tasks(m, 50.0, rng);
          const auto c = al::euclidean_costs(rs, ts);
          mrc_apps::Stopwatch clock;
          const auto opt = al::solve_hungarian(c);
          h_time.add(clock.elapsed_ms());
          h_cost.add(opt.total_cost);
          g_gap.add(100.0 * (greedy_cost(c) / opt.total_cost - 1.0));
          for (const auto mode : {al::BiddingMode::kGaussSeidel, al::BiddingMode::kJacobi}) {
            al::AuctionConfig acfg;
            acfg.mode = mode;
            mrc_apps::Stopwatch ac;
            const auto r = al::solve_auction(c, acfg);
            const double ms = ac.elapsed_ms();
            const double gap = 100.0 * (r.assignment.total_cost / opt.total_cost - 1.0);
            if (mode == al::BiddingMode::kGaussSeidel) {
              gs_gap.add(gap);
              gs_time.add(ms);
              gs_msgs.add(static_cast<double>(r.messages));
            } else {
              ja_gap.add(gap);
              ja_time.add(ms);
              ja_rounds.add(static_cast<double>(r.rounds));
              ja_msgs.add(static_cast<double>(r.messages));
            }
          }
        }
        static_rows.push_back(
            Json{{"case", name},
                 {"robots", n},
                 {"tasks", m},
                 {"hungarian", {{"cost", h_cost.json(3)}, {"runtime_ms", h_time.json(4)}}},
                 {"greedy", {{"gap_percent", g_gap.json(3)}}},
                 {"auction_gauss_seidel",
                  {{"gap_percent", gs_gap.json(5)},
                   {"runtime_ms", gs_time.json(4)},
                   {"messages", gs_msgs.json(0)}}},
                 {"auction_jacobi",
                  {{"gap_percent", ja_gap.json(5)},
                   {"runtime_ms", ja_time.json(4)},
                   {"rounds", ja_rounds.json(1)},
                   {"messages", ja_msgs.json(0)}}}});
      }
    }

    Json dynamic_rows = Json::array();
    const auto dyn = cfg.at("dynamic");
    for (const auto& penalty_json : dyn.at("switching_penalties")) {
      Sample mean_c, max_c, reassign;
      for (std::size_t s = 0; s < seeds; ++s) {
        const auto o =
            run_dynamic(dyn.value("robots", std::size_t{8}), dyn.value("arrival_rate", 0.6),
                        dyn.value("horizon", 120.0), penalty_json.get<double>(), 900 + s);
        mean_c.add(o.mean_completion);
        max_c.add(o.max_completion);
        reassign.add(static_cast<double>(o.reassignments));
      }
      dynamic_rows.push_back(Json{{"switching_penalty", penalty_json.get<double>()},
                                  {"mean_completion_time", mean_c.json(3)},
                                  {"max_completion_time", max_c.json(3)},
                                  {"reassignments", reassign.json(1)}});
    }

    Json multi_rows = Json::array();
    for (const auto& objective : {al::TeamObjective::kMiniSum, al::TeamObjective::kMiniMax}) {
      Sample total, makespan, runtime;
      for (std::size_t s = 0; s < seeds; ++s) {
        std::mt19937_64 rng(77 + s);
        const auto rs = random_robots(5, 30.0, rng);
        const auto ts = random_tasks(25, 30.0, rng);
        mrc_apps::Stopwatch clock;
        const auto plan = al::sequential_single_item_auction(rs, ts, {objective, 8});
        runtime.add(clock.elapsed_ms());
        total.add(plan.total_length);
        makespan.add(plan.max_length);
      }
      multi_rows.push_back(
          Json{{"objective", objective == al::TeamObjective::kMiniSum ? "minisum" : "minimax"},
               {"robots", 5},
               {"tasks", 25},
               {"total_length", total.json(3)},
               {"max_route_length", makespan.json(3)},
               {"runtime_ms", runtime.json(4)}});
    }

    Json result = mrc_apps::metadata("mrc_experiment_allocation_benchmark", cfg);
    result["static"] = static_rows;
    result["dynamic"] = dynamic_rows;
    result["multi_task"] = multi_rows;
    mrc_apps::write_result(args, result);
  });
}
