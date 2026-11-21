#include "multirobotcore/mapf/space_time_astar.hpp"

#include <algorithm>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace multirobotcore::mapf {
namespace {

struct SearchNode {
  int cell;
  int t;
  int conflicts;
  int parent;
};

struct OpenEntry {
  int f;
  int conflicts;
  int g;
  std::size_t id;
};

/// Orders the open list: lower f first, then fewer conflicts, then larger g
/// (deeper nodes, standard A* tie-breaking), then FIFO for determinism.
struct Worse {
  bool operator()(const OpenEntry& a, const OpenEntry& b) const noexcept {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    if (a.conflicts != b.conflicts) {
      return a.conflicts > b.conflicts;
    }
    if (a.g != b.g) {
      return a.g < b.g;
    }
    return a.id > b.id;
  }
};

}  // namespace

std::optional<Path> space_time_astar(const GridMap& map, Cell start, Cell goal,
                                     const ConstraintTable& constraints,
                                     std::span<const int> heuristic, LowLevelStats& stats,
                                     const LowLevelOptions& options) {
  if (heuristic.size() != map.num_cells()) {
    throw std::invalid_argument("space_time_astar: heuristic size must equal the number of cells");
  }
  if (!map.is_free(start) || !map.is_free(goal)) {
    return std::nullopt;
  }
  const int start_idx = map.index(start);
  const int goal_idx = map.index(goal);
  const int h0 = heuristic[static_cast<std::size_t>(start_idx)];
  if (h0 < 0 || constraints.is_permanently_blocked(goal) ||
      constraints.is_vertex_blocked(start, 0)) {
    return std::nullopt;
  }
  const int tc = constraints.max_time();
  const int horizon =
      options.max_time >= 0 ? options.max_time : tc + static_cast<int>(map.num_free_cells()) + 1;
  const int finish_after = constraints.latest_vertex_block(goal);

  // Beyond tc the problem is time-invariant: merge (v, t) for t > tc.
  auto state_key = [tc](int cell, int t) {
    return detail::vertex_key(cell, std::min(t, tc + 1));
  };

  std::vector<SearchNode> nodes;
  nodes.push_back({start_idx, 0, 0, -1});
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, Worse> open;
  open.push({h0, 0, 0, 0});
  ++stats.generated;
  std::unordered_set<std::uint64_t> closed;
  // Best (arrival time, conflicts) generated per state key. Merged states can
  // be reached at different times; an earlier arrival always dominates, and
  // conflicts only break ties between equal arrival times.
  std::unordered_map<std::uint64_t, std::pair<int, int>> best_generated;
  best_generated[state_key(start_idx, 0)] = {0, 0};

  while (!open.empty()) {
    const OpenEntry entry = open.top();
    open.pop();
    const SearchNode node = nodes[entry.id];
    if (!closed.insert(state_key(node.cell, node.t)).second) {
      continue;
    }
    ++stats.expansions;

    if (node.cell == goal_idx && node.t > finish_after) {
      Path path(static_cast<std::size_t>(node.t) + 1);
      int id = static_cast<int>(entry.id);
      while (id >= 0) {
        const SearchNode& n = nodes[static_cast<std::size_t>(id)];
        path[static_cast<std::size_t>(n.t)] = map.cell(n.cell);
        id = n.parent;
      }
      return path;
    }
    if (node.t >= horizon) {
      continue;
    }

    const Cell current = map.cell(node.cell);
    std::vector<Cell> successors = map.neighbors(current);
    successors.push_back(current);  // wait
    const int nt = node.t + 1;
    for (const Cell next : successors) {
      if (constraints.is_vertex_blocked(next, nt)) {
        continue;
      }
      if (next != current && constraints.is_edge_blocked(current, next, node.t)) {
        continue;
      }
      const int next_idx = map.index(next);
      const std::uint64_t key = state_key(next_idx, nt);
      if (closed.contains(key)) {
        continue;
      }
      const int conflicts =
          node.conflicts + (options.conflict_avoidance != nullptr
                                ? options.conflict_avoidance->conflicts(current, next, node.t)
                                : 0);
      const auto best = best_generated.find(key);
      if (best != best_generated.end()) {
        const auto [best_t, best_conflicts] = best->second;
        if (best_t < nt || (best_t == nt && best_conflicts <= conflicts)) {
          continue;
        }
      }
      best_generated[key] = {nt, conflicts};
      nodes.push_back({next_idx, nt, conflicts, static_cast<int>(entry.id)});
      open.push(
          {nt + heuristic[static_cast<std::size_t>(next_idx)], conflicts, nt, nodes.size() - 1});
      ++stats.generated;
    }
  }
  return std::nullopt;
}

}  // namespace multirobotcore::mapf
