// Cooperative frontier exploration on a shared map: one recorded run of the
// configured strategy plus a comparison of all strategies from the same start.
//
// Output: JSON consumed by scripts/render/render_exploration.py.
//
// mrc-modules: exploration io

#include "demo_common.hpp"

#include "multirobotcore/exploration/exploration.hpp"
#include "multirobotcore/io/json.hpp"

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ex = multirobotcore::exploration;
namespace io = multirobotcore::io;
using mrc_apps::Json;

namespace {

ex::Strategy parse_strategy(const std::string& name) {
  const auto s = ex::strategy_from_string(name);
  if (!s) {
    throw std::invalid_argument("unknown exploration strategy: " + name);
  }
  return *s;
}

ex::ExplorationConfig make_config(const Json& cfg, ex::Strategy strategy, double resolution) {
  ex::ExplorationConfig c;
  const auto& p = cfg.at("planner");
  c.planner.strategy = strategy;
  c.planner.information_weight = p.value("information_weight", 1.0);
  c.planner.cost_weight = p.value("cost_weight", 1.0);
  c.planner.discount_radius = p.value("discount_radius", 3.0) / resolution;  // [m] -> [cells]
  c.planner.min_cluster_size = p.value("min_cluster_size", std::size_t{3});
  c.sensor.range = cfg.at("sensor").value("range", 3.0);
  c.sensor.num_beams = cfg.at("sensor").value("beams", 180);
  c.replan_period = cfg.value("replan_period", std::size_t{8});
  c.max_steps = cfg.value("max_steps", std::size_t{2000});
  return c;
}

Json cell_json(const ex::GridCell& c) {
  return Json::array({c.x, c.y});
}

Json metrics_json(const ex::ExplorationResult& r, double step_seconds) {
  auto opt = [&](const std::optional<std::size_t>& s) {
    return s ? Json(io::round_to(static_cast<double>(*s) * step_seconds, 2)) : Json(nullptr);
  };
  double total = 0.0;
  for (const double d : r.distance) {
    total += d;
  }
  return Json{{"complete", r.complete},
              {"steps", r.steps},
              {"time_s", io::round_to(static_cast<double>(r.steps) * step_seconds, 2)},
              {"time_to_90_s", opt(r.steps_to_90)},
              {"time_to_95_s", opt(r.steps_to_95)},
              {"final_coverage", io::round_to(r.coverage.back(), 5)},
              {"total_distance_m", io::round_to(total, 2)},
              {"observed_cells", r.observed_cells},
              {"redundant_cells", r.redundant_cells},
              {"redundancy_ratio",
               io::round_to(r.observed_cells == 0 ? 0.0
                                                  : static_cast<double>(r.redundant_cells) /
                                                        static_cast<double>(r.observed_cells),
                            4)},
              {"replans", r.replans},
              {"messages", r.messages},
              {"bytes", r.bytes}};
}

Json coverage_json(const std::vector<double>& coverage) {
  Json out = Json::array();
  for (const double c : coverage) {
    out.push_back(io::round_to(c, 4));
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "exploration", "configs/exploration/exploration.json");
    const Json cfg = mrc_apps::load_config(args);
    const double resolution = cfg.value("resolution", 0.5);
    const auto map_path = mrc_apps::fs::path(MRC_SOURCE_DIR) / cfg.at("map").get<std::string>();
    const ex::OccupancyGrid truth = ex::OccupancyGrid::load_ascii(map_path, resolution);
    std::vector<ex::GridCell> starts;
    for (const auto& s : cfg.at("starts")) {
      starts.push_back({s.at(0).get<int>(), s.at(1).get<int>()});
    }
    const double step_seconds = resolution / cfg.value("robot_speed", 0.5);

    // Recorded run of the main strategy.
    const ex::Strategy main_strategy = parse_strategy(cfg.value("strategy", "utility_discount"));
    auto main_cfg = make_config(cfg, main_strategy, resolution);
    main_cfg.record_frames = true;
    ex::ExplorationSimulation sim(truth, starts, main_cfg);
    const ex::ExplorationResult run = sim.run();

    Json frames = Json::array();
    for (const auto& f : run.frames) {
      Json robots = Json::array();
      for (const auto& r : f.robots) {
        robots.push_back(cell_json(r));
      }
      Json targets = Json::array();
      for (const auto& t : f.targets) {
        targets.push_back(t ? cell_json(*t) : Json(nullptr));
      }
      Json known = Json::array();
      for (const auto& c : f.newly_known) {
        known.push_back(Json::array({truth.index(c), static_cast<int>(truth.at(c))}));
      }
      Json frontier = Json::array();
      for (const auto& c : f.frontier) {
        frontier.push_back(truth.index(c));
      }
      frames.push_back(Json{{"step", f.step},
                            {"robots", robots},
                            {"targets", targets},
                            {"known", known},
                            {"frontier", frontier},
                            {"coverage", io::round_to(f.coverage, 4)}});
    }

    // Strategy comparison from the same start.
    Json comparison = Json::object();
    for (const auto& name : cfg.at("compare_strategies")) {
      ex::ExplorationSimulation other(
          truth, starts, make_config(cfg, parse_strategy(name.get<std::string>()), resolution));
      const auto r = other.run();
      comparison[name.get<std::string>()] =
          Json{{"metrics", metrics_json(r, step_seconds)}, {"coverage", coverage_json(r.coverage)}};
    }

    // Ground truth as text rows (top row first, like the scenario file).
    Json rows = Json::array();
    for (int y = truth.height() - 1; y >= 0; --y) {
      std::string row;
      for (int x = 0; x < truth.width(); ++x) {
        row.push_back(truth.at({x, y}) == ex::Occupancy::kOccupied ? '#' : '.');
      }
      rows.push_back(row);
    }

    Json result = mrc_apps::metadata("mrc_demo_exploration", cfg);
    result["map"] = Json{{"width", truth.width()},
                         {"height", truth.height()},
                         {"resolution", resolution},
                         {"rows", rows}};
    result["step_seconds"] = step_seconds;
    result["sensor_range_cells"] = main_cfg.sensor.range / resolution;
    result["strategy"] = std::string(ex::to_string(main_strategy));
    result["metrics"] = metrics_json(run, step_seconds);
    result["coverage"] = coverage_json(run.coverage);
    result["frames"] = frames;
    result["comparison"] = comparison;
    mrc_apps::write_result(args, result);
  });
}
