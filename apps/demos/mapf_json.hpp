#pragma once

// JSON serialisation of MAPF maps, instances and planner results, shared by
// the MAPF demos and the MAPF experiment runner.

#include "multirobotcore/io/json.hpp"
#include "multirobotcore/mapf/conflicts.hpp"
#include "multirobotcore/mapf/grid_map.hpp"
#include "multirobotcore/mapf/instance.hpp"
#include "multirobotcore/mapf/planners.hpp"

#include <string>
#include <vector>

namespace mrc_apps::mapf_json {

using Json = multirobotcore::io::Json;
namespace mapf = multirobotcore::mapf;

inline Json cell(mapf::Cell c) {
  return Json::array({c.x, c.y});
}

inline Json cells(const std::vector<mapf::Cell>& list) {
  Json out = Json::array();
  for (const auto& c : list) {
    out.push_back(cell(c));
  }
  return out;
}

/// {"width", "height", "rows"} with rows listed top row first.
inline Json grid(const mapf::GridMap& map) {
  return Json{{"width", map.width()}, {"height", map.height()}, {"rows", map.to_ascii()}};
}

inline Json agents(const mapf::Instance& instance) {
  Json out = Json::array();
  for (const auto& a : instance.agents) {
    out.push_back(Json{{"start", cell(a.start)}, {"goal", cell(a.goal)}});
  }
  return out;
}

inline Json paths(const std::vector<mapf::Path>& list) {
  Json out = Json::array();
  for (const auto& p : list) {
    out.push_back(cells(p));
  }
  return out;
}

inline Json conflicts(const std::vector<mapf::Conflict>& list) {
  Json out = Json::array();
  for (const auto& c : list) {
    out.push_back(Json{{"type", c.type == mapf::ConflictType::kVertex ? "vertex" : "edge"},
                       {"agents", Json::array({c.agent1, c.agent2})},
                       {"cell", cell(c.cell1)},
                       {"cell2", cell(c.cell2)},
                       {"t", c.time}});
  }
  return out;
}

inline Json result(const mapf::PlannerResult& r, bool include_paths) {
  Json out{{"status", std::string(mapf::to_string(r.status))},
           {"sum_of_costs", r.sum_of_costs},
           {"makespan", r.makespan},
           {"conflicts", r.conflicts},
           {"runtime_ms", multirobotcore::io::round_to(r.runtime_ms, 3)},
           {"low_level_expansions", r.low_level_expansions},
           {"high_level_expanded", r.high_level_expanded},
           {"high_level_generated", r.high_level_generated},
           {"attempts", r.attempts},
           {"message", r.message}};
  if (include_paths) {
    out["paths"] = paths(r.paths);
    out["conflict_list"] = conflicts(mapf::find_conflicts(r.paths));
  }
  return out;
}

}  // namespace mrc_apps::mapf_json
