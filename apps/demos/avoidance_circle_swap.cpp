// Circle swap: N robots on a circle move to the antipodal points through the
// crowded centre. Runs ORCA, sampling RVO and sampling VO from identical
// initial conditions and records trajectories for the renderer.
//
// Output: JSON consumed by scripts/render/render_circle_swap.py.
//
// mrc-modules: avoidance simulation io

#include "avoidance_scenarios.hpp"
#include "demo_common.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace av = multirobotcore::avoidance;
using mrc_apps::Json;

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args = mrc_apps::parse_args(argc, argv, "avoidance_circle_swap",
                                           "configs/avoidance/circle_swap.json");
    const Json cfg = mrc_apps::load_config(args);
    const auto seed = cfg.value("seed", std::uint64_t{3});
    auto scenario = mrc_avoid::circle_swap(cfg.value("num_robots", std::size_t{20}),
                                           cfg.value("circle_radius", 8.0),
                                           cfg.value("placement_jitter", 0.05), seed);
    scenario.robot_radius = cfg.value("robot_radius", 0.3);
    scenario.max_speed = cfg.value("max_speed", 1.0);
    scenario.dt = cfg.value("dt", 0.05);
    scenario.duration = cfg.value("duration", 70.0);
    scenario.perturbation = cfg.value("preference_perturbation", 1e-3);
    scenario.goal_tolerance = cfg.value("goal_tolerance", 0.05);
    scenario.record_every = cfg.value("record_every", std::size_t{2});
    const auto recorded = cfg.value("recorded_methods", std::vector<std::string>{"orca"});

    Json methods = Json::object();
    for (const auto& name : cfg.value("methods", std::vector<std::string>{"orca", "rvo", "vo"})) {
      const auto method = av::parse_method(name);
      const auto config =
          mrc_avoid::avoidance_config(cfg.value("avoidance", Json::object()), method, scenario.dt);
      const bool record = std::find(recorded.begin(), recorded.end(), name) != recorded.end();
      const auto outcome = mrc_avoid::run(scenario, config, record);
      Json entry{{"metrics", mrc_avoid::metrics_json(outcome)},
                 {"collisions", mrc_avoid::collisions_json(outcome)}};
      if (record) {
        entry["trajectory"] = multirobotcore::io::frames_to_json(outcome.frames);
      }
      methods[name] = std::move(entry);
      std::cout << name << ": reached " << outcome.reached << "/" << scenario.starts.size()
                << ", collisions " << outcome.robot_collisions << ", makespan " << outcome.makespan
                << " s, min clearance " << outcome.min_pair_clearance << " m\n";
    }
    Json result = mrc_apps::metadata("mrc_demo_avoidance_circle_swap", cfg);
    result["scenario"] = mrc_avoid::scenario_json(scenario);
    result["methods"] = methods;
    mrc_apps::write_result(args, result);
  });
}
