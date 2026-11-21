// Why coordination matters: independent shortest paths versus prioritized
// planning versus Conflict-Based Search on a crafted instance.
//
// Output: JSON consumed by scripts/render/render_cbs.py.
//
// mrc-modules: mapf io

#include "demo_common.hpp"
#include "mapf_json.hpp"

#include "multirobotcore/mapf/conflict_based_search.hpp"
#include "multirobotcore/mapf/conflicts.hpp"
#include "multirobotcore/mapf/grid_map.hpp"
#include "multirobotcore/mapf/planners.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace mapf = multirobotcore::mapf;
namespace mj = mrc_apps::mapf_json;
using mrc_apps::Json;

int main(int argc, char** argv) {
  return mrc_apps::run_main([&] {
    const auto args =
        mrc_apps::parse_args(argc, argv, "mapf_conflicts", "configs/mapf/conflicts.json");
    const Json cfg = mrc_apps::load_config(args);

    mapf::Instance instance{
        mapf::GridMap::from_ascii(cfg.at("map").get<std::vector<std::string>>()), {}};
    for (const auto& a : cfg.at("agents")) {
      instance.agents.push_back({{a.at("start").at(0).get<int>(), a.at("start").at(1).get<int>()},
                                 {a.at("goal").at(0).get<int>(), a.at("goal").at(1).get<int>()}});
    }

    const auto independent = mapf::plan_independent(instance);
    mapf::PrioritizedOptions pp_options;
    pp_options.order = mapf::PriorityOrder::kShortestFirst;
    const auto prioritized = mapf::plan_prioritized(instance, pp_options);

    const Json& cbs_cfg = cfg.at("cbs");
    mapf::CbsOptions cbs_options;
    cbs_options.objective = cbs_cfg.value("objective", std::string("sum_of_costs")) == "makespan"
                                ? mapf::Objective::kMakespan
                                : mapf::Objective::kSumOfCosts;
    cbs_options.time_limit_s = cbs_cfg.value("time_limit_s", 60.0);
    cbs_options.max_high_level_nodes = cbs_cfg.value("max_high_level_nodes", std::size_t{500000});
    const auto cbs = mapf::ConflictBasedSearch(cbs_options).solve(instance);

    std::cout << "independent: " << independent.conflicts << " conflicts, SOC "
              << independent.sum_of_costs
              << "\nprioritized: " << mapf::to_string(prioritized.status)
              << "\nCBS: " << mapf::to_string(cbs.status) << ", SOC " << cbs.sum_of_costs
              << ", makespan " << cbs.makespan << ", CT nodes expanded " << cbs.high_level_expanded
              << '\n';

    Json result = mrc_apps::metadata("mrc_demo_mapf_conflicts", cfg);
    result["map"] = mj::grid(instance.map);
    result["agents"] = mj::agents(instance);
    result["independent"] = mj::result(independent, true);
    result["prioritized"] = mj::result(prioritized, prioritized.solved());
    result["cbs"] = mj::result(cbs, cbs.solved());
    mrc_apps::write_result(args, result);
  });
}
