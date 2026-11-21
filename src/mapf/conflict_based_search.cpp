#include "multirobotcore/mapf/conflict_based_search.hpp"

#include "multirobotcore/mapf/conflicts.hpp"
#include "multirobotcore/mapf/constraint_table.hpp"
#include "multirobotcore/mapf/space_time_astar.hpp"

#include "conflict_scan.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <vector>

namespace multirobotcore::mapf {
namespace {

struct Constraint {
  std::size_t agent{0};
  bool is_edge{false};
  Cell from;  ///< vertex constraint: the cell
  Cell to;    ///< edge constraint: destination
  int t{0};
};

using SharedPath = std::shared_ptr<const Path>;

/// For each timestep 0..cost of an agent's MDD (the union of all its
/// shortest constrained paths): the cell index if every such path is in the
/// same cell at that time, otherwise -1.
using Singletons = std::shared_ptr<const std::vector<int>>;

struct CtNode {
  int parent{-1};
  std::optional<Constraint> constraint;  ///< constraint added at this node
  std::vector<SharedPath> paths;
  std::vector<Singletons> singletons;  ///< lazily computed, shared with children
  std::size_t cost{0};
  std::size_t conflicts{0};
};

struct OpenEntry {
  std::size_t cost;
  std::size_t conflicts;
  std::size_t id;
};

struct Worse {
  bool operator()(const OpenEntry& a, const OpenEntry& b) const noexcept {
    if (a.cost != b.cost) {
      return a.cost > b.cost;
    }
    if (a.conflicts != b.conflicts) {
      return a.conflicts > b.conflicts;
    }
    return a.id > b.id;
  }
};

std::size_t node_cost(const std::vector<SharedPath>& paths, Objective objective) {
  std::size_t total = 0;
  for (const auto& p : paths) {
    const std::size_t c = path_cost(*p);
    total = objective == Objective::kSumOfCosts ? total + c : std::max(total, c);
  }
  return total;
}

std::size_t node_conflicts(const std::vector<SharedPath>& paths) {
  std::size_t count = 0;
  detail::scan_conflicts(
      paths.size(), [&](std::size_t i) -> const Path& { return *paths[i]; },
      [&](const Conflict& /*c*/) {
        ++count;
        return true;
      });
  return count;
}

std::optional<Conflict> node_first_conflict(const std::vector<SharedPath>& paths) {
  std::optional<Conflict> found;
  detail::scan_conflicts(
      paths.size(), [&](std::size_t i) -> const Path& { return *paths[i]; },
      [&](const Conflict& c) {
        found = c;
        return false;
      });
  return found;
}

std::vector<Path> materialise(const std::vector<SharedPath>& paths) {
  std::vector<Path> out;
  out.reserve(paths.size());
  for (const auto& p : paths) {
    out.push_back(*p);
  }
  return out;
}

}  // namespace

PlannerResult ConflictBasedSearch::solve(const Instance& instance) const {
  validate_instance(instance);
  const auto start_time = std::chrono::steady_clock::now();
  auto elapsed_s = [&] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
  };
  const GridMap& map = instance.map;
  const std::size_t n = instance.agents.size();

  std::vector<std::vector<int>> heuristics;
  heuristics.reserve(n);
  for (const Agent& a : instance.agents) {
    heuristics.push_back(map.distances_from(a.goal));
  }

  PlannerResult result;
  LowLevelStats ll_stats;
  auto finish = [&](PlannerStatus status, std::string message) {
    result.status = status;
    result.message = std::move(message);
    result.low_level_expansions = ll_stats.expansions;
    result.runtime_ms = elapsed_s() * 1000.0;
    return result;
  };

  std::deque<CtNode> nodes;  // deque: references stay valid as nodes are added

  // Constraints of `agent` on the CT branch ending at `node_id`, plus `extra`.
  auto build_table = [&](std::size_t agent, int node_id, const std::optional<Constraint>& extra) {
    ConstraintTable table(map);
    auto apply = [&](const Constraint& c) {
      if (c.agent != agent) {
        return;
      }
      if (c.is_edge) {
        table.add_edge(c.from, c.to, c.t);
      } else {
        table.add_vertex(c.from, c.t);
      }
    };
    if (extra) {
      apply(*extra);
    }
    for (int id = node_id; id >= 0; id = nodes[static_cast<std::size_t>(id)].parent) {
      if (const auto& c = nodes[static_cast<std::size_t>(id)].constraint) {
        apply(*c);
      }
    }
    return table;
  };

  // Low-level call for one agent under its branch constraints.
  auto plan_agent = [&](std::size_t agent, int node_id, const std::optional<Constraint>& extra,
                        const std::vector<SharedPath>& current) -> std::optional<Path> {
    const ConstraintTable table = build_table(agent, node_id, extra);
    LowLevelOptions options;
    std::optional<ConflictAvoidanceTable> cat;
    if (options_.use_conflict_avoidance && !current.empty()) {
      std::vector<Path> others;
      others.reserve(current.size());
      for (std::size_t a = 0; a < current.size(); ++a) {
        others.push_back(current[a] && a != agent ? *current[a] : Path{});
      }
      cat.emplace(map, others, agent);
      options.conflict_avoidance = &*cat;
    }
    return space_time_astar(map, instance.agents[agent].start, instance.agents[agent].goal, table,
                            heuristics[agent], ll_stats, options);
  };

  // Singleton layers of the agent's MDD at the node: forward reachability
  // within the cost bound, then backward pruning to the states that lie on
  // some shortest constrained path.
  auto singleton_layers = [&](std::size_t agent, int node_id, std::size_t cost) {
    const ConstraintTable table = build_table(agent, node_id, std::nullopt);
    const std::vector<int>& h = heuristics[agent];
    const int goal = map.index(instance.agents[agent].goal);
    const auto c = static_cast<int>(cost);
    auto successors = [&](int u, int t) {
      std::vector<int> out;
      const Cell cu = map.cell(u);
      std::vector<Cell> next = map.neighbors(cu);
      next.push_back(cu);
      for (const Cell v : next) {
        const int vi = map.index(v);
        if (table.is_vertex_blocked(v, t + 1) || (v != cu && table.is_edge_blocked(cu, v, t)) ||
            h[static_cast<std::size_t>(vi)] > c - (t + 1)) {
          continue;
        }
        out.push_back(vi);
      }
      return out;
    };
    std::vector<std::vector<int>> layers(cost + 1);
    layers[0] = {map.index(instance.agents[agent].start)};
    for (int t = 0; t < c; ++t) {
      std::vector<int> next;
      for (const int u : layers[static_cast<std::size_t>(t)]) {
        for (const int v : successors(u, t)) {
          next.push_back(v);
        }
      }
      std::sort(next.begin(), next.end());
      next.erase(std::unique(next.begin(), next.end()), next.end());
      layers[static_cast<std::size_t>(t) + 1] = std::move(next);
    }
    std::vector<std::vector<int>> keep(cost + 1);
    if (std::binary_search(layers[cost].begin(), layers[cost].end(), goal)) {
      keep[cost] = {goal};
    }
    for (int t = c - 1; t >= 0; --t) {
      const auto& later = keep[static_cast<std::size_t>(t) + 1];
      for (const int u : layers[static_cast<std::size_t>(t)]) {
        for (const int v : successors(u, t)) {
          if (std::binary_search(later.begin(), later.end(), v)) {
            keep[static_cast<std::size_t>(t)].push_back(u);
            break;
          }
        }
      }
    }
    auto out = std::make_shared<std::vector<int>>(cost + 1, -1);
    for (std::size_t t = 0; t <= cost; ++t) {
      if (keep[t].size() == 1) {
        (*out)[t] = keep[t].front();
      }
    }
    return Singletons(std::move(out));
  };

  // True if every shortest path of the agent under its constraints uses the
  // conflicting vertex/edge, so that any constraint resolving the conflict
  // increases the agent's cost.
  auto forced = [&](std::size_t agent, const Conflict& conflict, bool is_agent1, int node_id) {
    CtNode& node = nodes[static_cast<std::size_t>(node_id)];
    const std::size_t cost = path_cost(*node.paths[agent]);
    const auto t = static_cast<std::size_t>(conflict.time);
    if (conflict.type == ConflictType::kVertex && t >= cost) {
      return true;  // parked at its goal: it must arrive later instead
    }
    if (!node.singletons[agent]) {
      node.singletons[agent] = singleton_layers(agent, node_id, cost);
    }
    const auto& layer = *node.singletons[agent];
    if (conflict.type == ConflictType::kVertex) {
      return layer[t] == map.index(conflict.cell1);
    }
    if (t + 1 > cost) {
      return false;
    }
    const Cell from = is_agent1 ? conflict.cell1 : conflict.cell2;
    const Cell to = is_agent1 ? conflict.cell2 : conflict.cell1;
    return layer[t] == map.index(from) && layer[t + 1] == map.index(to);
  };

  // Conflict to split: cardinal before semi-cardinal before non-cardinal,
  // earliest first within a class.
  auto choose_conflict = [&](int node_id) -> std::optional<Conflict> {
    const auto& paths = nodes[static_cast<std::size_t>(node_id)].paths;
    if (!options_.prioritize_cardinal_conflicts) {
      return node_first_conflict(paths);
    }
    std::vector<Conflict> all;
    detail::scan_conflicts(
        paths.size(), [&](std::size_t i) -> const Path& { return *paths[i]; },
        [&](const Conflict& c) {
          all.push_back(c);
          return true;
        });
    if (all.empty()) {
      return std::nullopt;
    }
    int best_class = -1;
    std::size_t best = 0;
    for (std::size_t k = 0; k < all.size(); ++k) {
      const int cls = (forced(all[k].agent1, all[k], true, node_id) ? 1 : 0) +
                      (forced(all[k].agent2, all[k], false, node_id) ? 1 : 0);
      if (cls > best_class) {
        best_class = cls;
        best = k;
        if (cls == 2) {
          break;
        }
      }
    }
    return all[best];
  };

  // Root: individually shortest paths (ties broken towards fewer conflicts
  // with the agents planned before).
  CtNode root;
  root.paths.resize(n);
  root.singletons.resize(n);
  for (std::size_t a = 0; a < n; ++a) {
    auto path = plan_agent(a, -1, std::nullopt, root.paths);
    if (!path) {
      return finish(PlannerStatus::kNoSolution,
                    "agent " + std::to_string(a) + " cannot reach its goal");
    }
    root.paths[a] = std::make_shared<const Path>(std::move(*path));
  }
  root.cost = node_cost(root.paths, options_.objective);
  root.conflicts = node_conflicts(root.paths);
  nodes.push_back(std::move(root));
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, Worse> open;
  open.push({nodes[0].cost, nodes[0].conflicts, 0});
  result.high_level_generated = 1;

  while (!open.empty()) {
    if (elapsed_s() > options_.time_limit_s) {
      return finish(PlannerStatus::kTimeout, "time limit reached");
    }
    if (result.high_level_expanded >= options_.max_high_level_nodes) {
      return finish(PlannerStatus::kNodeLimit, "high-level node limit reached");
    }
    const OpenEntry entry = open.top();
    open.pop();
    const auto node_id = static_cast<int>(entry.id);
    const std::optional<Conflict> conflict = choose_conflict(node_id);
    if (!conflict) {
      result.paths = materialise(nodes[entry.id].paths);
      result.sum_of_costs = sum_of_costs(result.paths);
      result.makespan = makespan(result.paths);
      result.conflicts = 0;
      return finish(PlannerStatus::kSolved, "");
    }
    ++result.high_level_expanded;

    std::array<Constraint, 2> branches{};
    if (conflict->type == ConflictType::kVertex) {
      branches[0] = {conflict->agent1, false, conflict->cell1, conflict->cell1, conflict->time};
      branches[1] = {conflict->agent2, false, conflict->cell1, conflict->cell1, conflict->time};
    } else {
      // agent1 moves cell1 -> cell2, agent2 moves cell2 -> cell1 at time t.
      branches[0] = {conflict->agent1, true, conflict->cell1, conflict->cell2, conflict->time};
      branches[1] = {conflict->agent2, true, conflict->cell2, conflict->cell1, conflict->time};
    }
    for (const Constraint& c : branches) {
      const std::vector<SharedPath>& parent_paths = nodes[entry.id].paths;
      auto path = plan_agent(c.agent, node_id, c, parent_paths);
      if (!path) {
        continue;  // no path satisfies this branch's constraints: prune
      }
      CtNode child;
      child.parent = node_id;
      child.constraint = c;
      child.paths = parent_paths;
      child.paths[c.agent] = std::make_shared<const Path>(std::move(*path));
      child.singletons = nodes[entry.id].singletons;
      child.singletons[c.agent] = nullptr;  // replanned: recompute on demand
      child.cost = node_cost(child.paths, options_.objective);
      child.conflicts = node_conflicts(child.paths);
      nodes.push_back(std::move(child));
      open.push({nodes.back().cost, nodes.back().conflicts, nodes.size() - 1});
      ++result.high_level_generated;
    }
  }
  return finish(PlannerStatus::kNoSolution, "constraint tree exhausted");
}

}  // namespace multirobotcore::mapf
