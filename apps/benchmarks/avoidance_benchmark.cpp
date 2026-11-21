// Collision-avoidance experiment: VO / RVO / ORCA x robot count x scenario
// (circle swap, random crossing, doorway). Every run uses the deterministic
// simulator; metrics: success, contacts, minimum clearance, time to goal,
// path efficiency, velocity deviation, stalls and computation time.
//
// Output: JSON consumed by scripts/benchmark/avoidance_report.py.
//
// mrc-modules: avoidance simulation io

#include "../demos/avoidance_scenarios.hpp"
#include "demo_common.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace av = multirobotcore::avoidance;
using mrc_apps::Json;

namespace {

mrc_avoid::Scenario build(const std::string& name, std::size_t n, const Json& cfg,
                          std::uint64_t seed) {
  const double radius = cfg.value("robot_radius", 0.25);
  mrc_avoid::Scenario s;
  if (name == "circle_swap") {
    // About one metre of circumference per robot, at least a 4 m radius.
    const double ring = std::max(4.0, static_cast<double>(n) * 1.0 / mrc_avoid::mrc::math::kTwoPi);
    s = mrc_avoid::circle_swap(n, ring, 0.05, seed);
    s.duration = 2.0 * ring / cfg.value("max_speed", 1.0) + 40.0;
  } else if (name == "random_crossing") {
    const double side = 1.8 * std::sqrt(static_cast<double>(n)) + 4.0;
    s = mrc_avoid::random_crossing(n, side, 2.0 * radius + 0.4, seed);
    s.duration = 1.5 * side / cfg.value("max_speed", 1.0) + 40.0;
  } else if (name == "doorway") {
    s = mrc_avoid::doorway(n, cfg.value("door_width", 1.2), false, seed);
    s.duration = 20.0 + 2.0 * static_cast<double>(n);
  } else {
    throw std::invalid_argument("unknown scenario " + name);
  }
  s.robot_radius = radius;
  s.max_speed = cfg.value("max_speed", 1.0);
  s.dt = cfg.value("dt", 0.05);
  s.perturbation = cfg.value("preference_perturbation", 0.05);
  s.goal_tolerance = cfg.value("goal_tolerance", 0.1);
  return s;
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "avoidance_benchmark", "configs/avoidance/benchmark.json");
    const Json cfg = mrc_apps::load_config(args);
    const auto seed = cfg.value("seed", std::uint64_t{11});
    const auto methods = cfg.value("methods", std::vector<std::string>{"vo", "rvo", "orca"});
    const auto counts = cfg.value("robot_counts", std::vector<std::size_t>{5, 10, 20, 50});
    const auto orca_counts = cfg.value("orca_only_counts", std::vector<std::size_t>{});
    Json records = Json::array();
    for (const auto& scenario_name :
         cfg.value("scenarios", std::vector<std::string>{"circle_swap"})) {
      std::vector<std::pair<std::string, std::size_t>> runs;
      for (const auto n : counts) {
        for (const auto& m : methods) {
          runs.emplace_back(m, n);
        }
      }
      for (const auto n : orca_counts) {
        runs.emplace_back("orca", n);
      }
      for (const auto& [method_name, n] : runs) {
        const auto scenario = build(scenario_name, n, cfg, seed);
        const auto config = mrc_avoid::avoidance_config(cfg.value("avoidance", Json::object()),
                                                        av::parse_method(method_name), scenario.dt);
        const mrc_apps::Stopwatch watch;
        const auto outcome = mrc_avoid::run(scenario, config, false);
        Json record{{"scenario", scenario_name}, {"method", method_name}, {"robots", n}};
        record["metrics"] = mrc_avoid::metrics_json(outcome);
        record["wall_time_s"] = multirobotcore::io::round_to(watch.elapsed_ms() / 1000.0, 3);
        std::cout << scenario_name << " " << method_name << " n=" << n << ": reached "
                  << outcome.reached << "/" << n << " contacts " << outcome.robot_collisions << "+"
                  << outcome.obstacle_collisions << " makespan " << outcome.makespan << " s"
                  << (outcome.stalled ? " STALL" : "") << '\n';
        records.push_back(std::move(record));
      }
    }
    Json result = mrc_apps::metadata("mrc_experiment_avoidance_benchmark", cfg);
    result["records"] = records;
    mrc_apps::write_result(args, result);
  });
}
