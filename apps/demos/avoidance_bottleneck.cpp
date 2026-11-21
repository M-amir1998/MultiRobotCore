// Bottlenecks for local collision avoidance: a doorway passed in one
// direction, a doorway with counterflow and a corridor swap that admits a
// single robot. Shows queueing, congestion and where purely local avoidance
// stalls (detected by the simulator's stall heuristic).
//
// Output: JSON consumed by scripts/render/render_bottleneck.py.
//
// mrc-modules: avoidance simulation io

#include "avoidance_scenarios.hpp"
#include "demo_common.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

namespace av = multirobotcore::avoidance;
using mrc_apps::Json;

namespace {

mrc_avoid::Scenario make_scenario(const Json& s, std::uint64_t seed) {
  const auto type = s.at("type").get<std::string>();
  if (type == "doorway" || type == "doorway_counterflow") {
    return mrc_avoid::doorway(s.value("num_robots", std::size_t{16}), s.value("door_width", 1.2),
                              type == "doorway_counterflow", seed);
  }
  if (type == "corridor_swap") {
    return mrc_avoid::corridor_swap(s.value("per_side", std::size_t{3}),
                                    s.value("corridor_width", 0.9), seed);
  }
  throw std::invalid_argument("unknown bottleneck scenario: " + type);
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args = mrc_apps::parse_args(argc, argv, "avoidance_bottleneck",
                                           "configs/avoidance/bottleneck.json");
    const Json cfg = mrc_apps::load_config(args);
    const auto seed = cfg.value("seed", std::uint64_t{5});
    const auto method = av::parse_method(cfg.value("method", std::string("orca")));
    Json scenarios = Json::array();
    for (const auto& spec : cfg.at("scenarios")) {
      auto scenario = make_scenario(spec, seed);
      scenario.robot_radius = cfg.value("robot_radius", 0.25);
      scenario.max_speed = cfg.value("max_speed", 1.0);
      scenario.dt = cfg.value("dt", 0.05);
      scenario.duration = cfg.value("duration", 60.0);
      scenario.perturbation = cfg.value("preference_perturbation", 0.05);
      scenario.goal_tolerance = cfg.value("goal_tolerance", 0.1);
      scenario.record_every = cfg.value("record_every", std::size_t{2});
      scenario.stall_window = cfg.value("stall_window", 6.0);
      const auto config =
          mrc_avoid::avoidance_config(cfg.value("avoidance", Json::object()), method, scenario.dt);
      const auto outcome = mrc_avoid::run(scenario, config, true);
      std::cout << scenario.name << ": reached " << outcome.reached << "/" << scenario.starts.size()
                << ", collisions " << outcome.robot_collisions << "+" << outcome.obstacle_collisions
                << ", makespan " << outcome.makespan << " s, stalled " << outcome.stalled << '\n';
      Json entry = mrc_avoid::scenario_json(scenario);
      entry["method"] = std::string(av::to_string(method));
      entry["metrics"] = mrc_avoid::metrics_json(outcome);
      entry["collisions"] = mrc_avoid::collisions_json(outcome);
      Json arrivals = Json::array();
      for (const double t : outcome.arrival_times) {
        arrivals.push_back(mrc_avoid::rounded_or_null(t, 2));
      }
      entry["arrival_times"] = arrivals;
      entry["obstacle_discs"] = Json::array();
      for (const auto& d : av::obstacle_discs(scenario.obstacles, config.obstacle_disc_spacing)) {
        entry["obstacle_discs"].push_back(multirobotcore::io::to_json(d));
      }
      entry["trajectory"] = multirobotcore::io::frames_to_json(outcome.frames);
      scenarios.push_back(std::move(entry));
    }
    Json result = mrc_apps::metadata("mrc_demo_avoidance_bottleneck", cfg);
    result["scenarios"] = scenarios;
    mrc_apps::write_result(args, result);
  });
}
