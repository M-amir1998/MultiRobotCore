// Exploration strategy comparison: maps x robot counts x strategies x seeds.
//
// Robots start on random free cells inside a start disc (seeded), all with
// the same shared-map configuration; only the assignment strategy varies.
//
// mrc-modules: exploration io

#include "demo_common.hpp"

#include "multirobotcore/exploration/exploration.hpp"
#include "multirobotcore/io/json.hpp"

#include <algorithm>
#include <random>
#include <string>
#include <vector>

namespace ex = multirobotcore::exploration;
namespace io = multirobotcore::io;
using mrc_apps::Json;

namespace {

std::vector<ex::GridCell> random_starts(const ex::OccupancyGrid& map, ex::GridCell center,
                                        int radius, std::size_t count, std::uint64_t seed) {
  std::vector<ex::GridCell> free_cells;
  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const ex::GridCell c{center.x + dx, center.y + dy};
      if (dx * dx + dy * dy <= radius * radius && map.in_bounds(c) &&
          map.at(c) == ex::Occupancy::kFree) {
        free_cells.push_back(c);
      }
    }
  }
  if (free_cells.size() < count) {
    throw std::runtime_error("not enough free start cells");
  }
  std::mt19937_64 rng(seed);
  std::shuffle(free_cells.begin(), free_cells.end(), rng);
  free_cells.resize(count);
  return free_cells;
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args = mrc_apps::parse_args(argc, argv, "exploration_benchmark",
                                           "configs/exploration/benchmark.json");
    const Json cfg = mrc_apps::load_config(args);
    const double resolution = cfg.value("resolution", 0.5);
    const double step_seconds = resolution / cfg.value("robot_speed", 0.5);
    Json runs = Json::array();
    for (const auto& map_cfg : cfg.at("maps")) {
      const auto map = ex::OccupancyGrid::load_ascii(
          mrc_apps::fs::path(MRC_SOURCE_DIR) / map_cfg.at("file").get<std::string>(), resolution);
      const ex::GridCell center{map_cfg.at("start").at(0).get<int>(),
                                map_cfg.at("start").at(1).get<int>()};
      for (const auto& robots : cfg.at("robot_counts")) {
        for (const auto& seed : cfg.at("seeds")) {
          const auto starts = random_starts(map, center, cfg.value("start_radius", 3),
                                            robots.get<std::size_t>(), seed.get<std::uint64_t>());
          for (const auto& name : cfg.at("strategies")) {
            ex::ExplorationConfig c;
            c.planner.strategy = *ex::strategy_from_string(name.get<std::string>());
            c.planner.discount_radius = cfg.value("sensor_range", 3.0) / resolution;
            c.planner.min_cluster_size = cfg.value("min_cluster_size", std::size_t{3});
            c.sensor.range = cfg.value("sensor_range", 3.0);
            c.sensor.num_beams = cfg.value("beams", 180);
            c.replan_period = cfg.value("replan_period", std::size_t{8});
            c.max_steps = cfg.value("max_steps", std::size_t{3000});
            mrc_apps::Stopwatch watch;
            ex::ExplorationSimulation sim(map, starts, c);
            const auto r = sim.run();
            const double runtime = watch.elapsed_ms();
            double distance = 0.0;
            for (const double d : r.distance) {
              distance += d;
            }
            auto seconds = [&](const std::optional<std::size_t>& s) {
              return s ? Json(static_cast<double>(*s) * step_seconds) : Json(nullptr);
            };
            runs.push_back(Json{
                {"map", map_cfg.at("name")},
                {"robots", robots},
                {"seed", seed},
                {"strategy", name},
                {"complete", r.complete},
                {"time_s", static_cast<double>(r.steps) * step_seconds},
                {"time_to_90_s", seconds(r.steps_to_90)},
                {"time_to_95_s", seconds(r.steps_to_95)},
                {"distance_m", io::round_to(distance, 2)},
                {"redundancy_ratio",
                 io::round_to(static_cast<double>(r.redundant_cells) /
                                  static_cast<double>(std::max<std::size_t>(r.observed_cells, 1)),
                              4)},
                {"messages", r.messages},
                {"bytes", r.bytes},
                {"runtime_ms", io::round_to(runtime, 3)}});
          }
        }
      }
    }
    Json result = mrc_apps::metadata("mrc_experiment_exploration_benchmark", cfg);
    result["step_seconds"] = step_seconds;
    result["runs"] = runs;
    mrc_apps::write_result(args, result);
  });
}
