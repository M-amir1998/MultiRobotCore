// Warehouse multi-agent path finding: robots travel from station columns to
// shelf access points through one-cell-wide aisles. Independent planning,
// prioritized planning and CBS are run on the same instance.
//
// Output: JSON consumed by scripts/render/render_mapf_warehouse.py.
//
// mrc-modules: mapf io

#include "demo_common.hpp"
#include "mapf_json.hpp"

#include "multirobotcore/mapf/conflict_based_search.hpp"
#include "multirobotcore/mapf/generators.hpp"
#include "multirobotcore/mapf/planners.hpp"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <random>
#include <string>

namespace mapf = multirobotcore::mapf;
namespace mj = mrc_apps::mapf_json;
using mrc_apps::Json;

namespace {

mapf::WarehouseLayout layout_from(const Json& j) {
  mapf::WarehouseLayout l;
  l.shelf_block_columns = j.value("shelf_block_columns", l.shelf_block_columns);
  l.shelf_block_rows = j.value("shelf_block_rows", l.shelf_block_rows);
  l.shelf_length = j.value("shelf_length", l.shelf_length);
  l.shelf_depth = j.value("shelf_depth", l.shelf_depth);
  l.aisle_width = j.value("aisle_width", l.aisle_width);
  l.station_margin = j.value("station_margin", l.station_margin);
  return l;
}

mapf::PriorityOrder order_from(const std::string& name) {
  if (name == "index") {
    return mapf::PriorityOrder::kIndex;
  }
  if (name == "longest_first") {
    return mapf::PriorityOrder::kLongestFirst;
  }
  if (name == "random") {
    return mapf::PriorityOrder::kRandom;
  }
  return mapf::PriorityOrder::kShortestFirst;
}

}  // namespace

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "mapf_warehouse", "configs/mapf/warehouse.json");
    const Json cfg = mrc_apps::load_config(args);
    const auto seed = cfg.value("seed", std::uint64_t{11});
    std::mt19937_64 rng(seed);

    const mapf::Warehouse wh = mapf::make_warehouse(layout_from(cfg.at("layout")));
    const auto n = cfg.value("num_agents", std::size_t{24});
    const mapf::Instance instance = mapf::random_instance(wh.map, n, wh.stations, wh.pickups, rng);

    const auto independent = mapf::plan_independent(instance);
    const Json& pp_cfg = cfg.at("prioritized");
    mapf::PrioritizedOptions pp_options;
    pp_options.order = order_from(pp_cfg.value("order", std::string("shortest_first")));
    pp_options.max_attempts = pp_cfg.value("max_attempts", std::size_t{20});
    pp_options.seed = seed;
    const auto prioritized = mapf::plan_prioritized(instance, pp_options);

    const Json& cbs_cfg = cfg.at("cbs");
    mapf::CbsOptions cbs_options;
    cbs_options.time_limit_s = cbs_cfg.value("time_limit_s", 60.0);
    cbs_options.max_high_level_nodes = cbs_cfg.value("max_high_level_nodes", std::size_t{300000});
    const auto cbs = mapf::ConflictBasedSearch(cbs_options).solve(instance);

    // CBS scales poorly in single-lane aisles: also compare the optimal CBS
    // solution with prioritized planning on a subset of the robots.
    const auto subset_size = std::min(n, cfg.value("subset_agents", std::size_t{12}));
    mapf::Instance subset{instance.map, {}};
    subset.agents.assign(instance.agents.begin(),
                         instance.agents.begin() + static_cast<std::ptrdiff_t>(subset_size));
    const auto subset_pp = mapf::plan_prioritized(subset, pp_options);
    const auto subset_cbs = mapf::ConflictBasedSearch(cbs_options).solve(subset);

    std::cout << "warehouse " << wh.map.width() << "x" << wh.map.height() << ", " << n
              << " robots\nindependent: " << independent.conflicts << " conflicts, SOC "
              << independent.sum_of_costs
              << "\nprioritized: " << mapf::to_string(prioritized.status) << " SOC "
              << prioritized.sum_of_costs << " (attempts " << prioritized.attempts
              << ")\nCBS: " << mapf::to_string(cbs.status) << " SOC " << cbs.sum_of_costs
              << " makespan " << cbs.makespan << " CT expanded " << cbs.high_level_expanded
              << " in " << cbs.runtime_ms << " ms\nsubset of " << subset_size
              << ": prioritized SOC " << subset_pp.sum_of_costs << ", CBS "
              << mapf::to_string(subset_cbs.status) << " SOC " << subset_cbs.sum_of_costs << '\n';

    Json result = mrc_apps::metadata("mrc_demo_mapf_warehouse", cfg);
    result["map"] = mj::grid(wh.map);
    result["stations"] = mj::cells(wh.stations);
    result["pickups"] = mj::cells(wh.pickups);
    result["agents"] = mj::agents(instance);
    result["world"] = cfg.at("world");
    result["independent"] = mj::result(independent, true);
    result["prioritized"] = mj::result(prioritized, prioritized.solved());
    result["cbs"] = mj::result(cbs, cbs.solved());
    result["subset"] = Json{{"num_agents", subset_size},
                            {"prioritized", mj::result(subset_pp, subset_pp.solved())},
                            {"cbs", mj::result(subset_cbs, subset_cbs.solved())}};
    mrc_apps::write_result(args, result);
  });
}
