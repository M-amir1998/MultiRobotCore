// MAPF experiment runner: independent A*, prioritized planning (single order
// and random restarts) and Conflict-Based Search across map families and team
// sizes. Writes one JSON row per (map, team size, seed, planner).
//
// Consumed by scripts/benchmark/mapf_benchmark.py.
//
// mrc-modules: mapf io

#include "demo_common.hpp"

#include "multirobotcore/mapf/conflict_based_search.hpp"
#include "multirobotcore/mapf/generators.hpp"
#include "multirobotcore/mapf/planners.hpp"

#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace mapf = multirobotcore::mapf;
namespace io = multirobotcore::io;
using mrc_apps::Json;

namespace {

struct MapCase {
  std::string name;
  mapf::GridMap map;
  std::vector<mapf::Cell> starts;  // empty: largest component
  std::vector<mapf::Cell> goals;
};

MapCase build_map(const Json& j, std::uint64_t seed) {
  const std::string type = j.at("type").get<std::string>();
  MapCase c{j.at("name").get<std::string>(), {}, {}, {}};
  std::mt19937_64 rng(seed * 7919U + 17U);
  if (type == "open") {
    c.map = mapf::make_open_map(j.at("width").get<int>(), j.at("height").get<int>());
  } else if (type == "maze") {
    c.map = mapf::make_maze(j.at("width").get<int>(), j.at("height").get<int>(),
                            j.value("loop_fraction", 0.15), rng);
  } else if (type == "corridor") {
    c.map = mapf::make_corridor_map(j.at("room_width").get<int>(), j.at("room_height").get<int>(),
                                    j.at("corridor_length").get<int>());
  } else if (type == "warehouse") {
    const Json& l = j.at("layout");
    mapf::WarehouseLayout layout;
    layout.shelf_block_columns = l.value("shelf_block_columns", layout.shelf_block_columns);
    layout.shelf_block_rows = l.value("shelf_block_rows", layout.shelf_block_rows);
    layout.shelf_length = l.value("shelf_length", layout.shelf_length);
    layout.shelf_depth = l.value("shelf_depth", layout.shelf_depth);
    layout.aisle_width = l.value("aisle_width", layout.aisle_width);
    layout.station_margin = l.value("station_margin", layout.station_margin);
    auto wh = mapf::make_warehouse(layout);
    c.map = std::move(wh.map);
    c.starts = std::move(wh.stations);
    c.goals = std::move(wh.pickups);
  } else {
    throw std::invalid_argument("unknown map type: " + type);
  }
  return c;
}

Json row(const MapCase& m, std::size_t agents, std::uint64_t seed, const std::string& planner,
         const mapf::PlannerResult& r) {
  const bool success = r.status == mapf::PlannerStatus::kSolved;
  return Json{{"map", m.name},
              {"width", m.map.width()},
              {"height", m.map.height()},
              {"agents", agents},
              {"seed", seed},
              {"planner", planner},
              {"status", std::string(mapf::to_string(r.status))},
              {"success", success},
              {"runtime_ms", io::round_to(r.runtime_ms, 3)},
              {"sum_of_costs", r.sum_of_costs},
              {"makespan", r.makespan},
              {"conflicts", r.conflicts},
              {"hl_expanded", r.high_level_expanded},
              {"hl_generated", r.high_level_generated},
              {"ll_expansions", r.low_level_expansions},
              {"attempts", r.attempts}};
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "mapf_benchmark", "configs/mapf/benchmark.json");
    const Json cfg = mrc_apps::load_config(args);
    const auto seeds = cfg.at("seeds").get<std::vector<std::uint64_t>>();
    mapf::CbsOptions cbs_options;
    cbs_options.time_limit_s = cfg.value("cbs_time_limit_s", 3.0);
    const auto restarts = cfg.value("restart_attempts", std::size_t{10});

    Json rows = Json::array();
    for (const Json& map_cfg : cfg.at("maps")) {
      for (const auto agents : map_cfg.at("agents").get<std::vector<std::size_t>>()) {
        for (const auto seed : seeds) {
          const MapCase m = build_map(map_cfg, seed);
          std::mt19937_64 rng(seed);
          const mapf::Instance instance =
              m.starts.empty() ? mapf::random_instance(m.map, agents, rng)
                               : mapf::random_instance(m.map, agents, m.starts, m.goals, rng);

          rows.push_back(row(m, agents, seed, "independent", mapf::plan_independent(instance)));

          mapf::PrioritizedOptions single;
          single.order = mapf::PriorityOrder::kShortestFirst;
          rows.push_back(
              row(m, agents, seed, "prioritized", mapf::plan_prioritized(instance, single)));

          mapf::PrioritizedOptions restart = single;
          restart.max_attempts = restarts;
          restart.seed = seed;
          rows.push_back(row(m, agents, seed, "prioritized_restarts",
                             mapf::plan_prioritized(instance, restart)));

          const auto cbs = mapf::ConflictBasedSearch(cbs_options).solve(instance);
          rows.push_back(row(m, agents, seed, "cbs", cbs));
          std::cout << m.name << " agents=" << agents << " seed=" << seed
                    << " cbs=" << mapf::to_string(cbs.status) << " (" << cbs.runtime_ms << " ms)\n";
        }
      }
    }
    Json result = mrc_apps::metadata("mrc_experiment_mapf_benchmark", cfg);
    result["rows"] = rows;
    mrc_apps::write_result(args, result);
  });
}
