#pragma once

/// @file constraint_table.hpp
/// Space-time restrictions for one agent's low-level search.
///
/// The same table serves both coordination schemes:
///  - Conflict-Based Search adds the vertex/edge constraints of a CT node;
///  - Prioritized Planning reserves the paths of higher-priority agents
///    (reserve_path), including their parking at the goal.

#include "multirobotcore/mapf/grid_map.hpp"
#include "multirobotcore/mapf/instance.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace multirobotcore::mapf {

namespace detail {

/// Move from cell index `from` to `to` departing at time t.
struct TimedMove {
  int from{0};
  int to{0};
  int t{0};
  friend bool operator==(const TimedMove&, const TimedMove&) = default;
};

struct TimedMoveHash {
  std::size_t operator()(const TimedMove& m) const noexcept;
};

/// Key for (cell index, time) pairs.
[[nodiscard]] constexpr std::uint64_t vertex_key(int index, int t) noexcept {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(t)) << 32U) |
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(index));
}

}  // namespace detail

class ConstraintTable {
 public:
  explicit ConstraintTable(const GridMap& map);

  /// Forbids occupying `cell` at time t.
  void add_vertex(Cell cell, int t);
  /// Forbids moving from -> to between t and t + 1.
  void add_edge(Cell from, Cell to, int t);
  /// Forbids occupying `cell` at every time >= t (another agent parked there).
  void add_permanent(Cell cell, int t);

  /// Reserves a higher-priority agent's path: its cell at every timestep,
  /// the reverse of each of its moves (no swaps), and its final cell from
  /// arrival onwards.
  void reserve_path(const Path& path);

  [[nodiscard]] bool is_vertex_blocked(Cell cell, int t) const;
  [[nodiscard]] bool is_edge_blocked(Cell from, Cell to, int t) const;

  /// Latest time at which `cell` is blocked by a vertex constraint; -1 if
  /// never. An agent may only finish at its goal after this time.
  [[nodiscard]] int latest_vertex_block(Cell cell) const;

  /// True when `cell` is blocked forever from some time on.
  [[nodiscard]] bool is_permanently_blocked(Cell cell) const;

  /// Largest time referenced by any constraint (-1 if empty). Beyond it the
  /// problem is time-invariant.
  [[nodiscard]] int max_time() const noexcept { return max_time_; }

  [[nodiscard]] bool empty() const noexcept {
    return vertex_.empty() && edge_.empty() && permanent_.empty();
  }

 private:
  const GridMap* map_;
  std::unordered_set<std::uint64_t> vertex_;
  std::unordered_set<detail::TimedMove, detail::TimedMoveHash> edge_;
  std::unordered_map<int, int> latest_vertex_;
  std::unordered_map<int, int> permanent_;
  int max_time_{-1};
};

/// Counts how many conflicts a candidate move would create with a fixed set of
/// other agents' paths. Used only to break ties between equally short paths
/// (fewer conflicts first), which never changes the cost of the result.
class ConflictAvoidanceTable {
 public:
  ConflictAvoidanceTable(const GridMap& map, const std::vector<Path>& paths,
                         std::size_t skip_agent);

  /// Conflicts created by moving from -> to between t and t + 1 (a vertex
  /// conflict at the destination at t + 1 or a swap).
  [[nodiscard]] int conflicts(Cell from, Cell to, int t) const;

 private:
  const GridMap* map_;
  std::unordered_map<std::uint64_t, int> occupancy_;  // (cell, t) -> agents
  std::unordered_map<detail::TimedMove, int, detail::TimedMoveHash> moves_;
  std::unordered_map<int, std::vector<int>> parked_;  // cell -> park start times
};

}  // namespace multirobotcore::mapf
