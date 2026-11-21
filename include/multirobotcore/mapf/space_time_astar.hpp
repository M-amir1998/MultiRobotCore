#pragma once

/// @file space_time_astar.hpp
/// Space-time A* (Silver 2005) over states (cell, t) with move and wait
/// actions of unit cost, honouring a ConstraintTable.
///
/// Correctness details:
///  - The heuristic is the exact single-agent distance (BFS), which is
///    admissible and consistent for unit-cost moves and waits.
///  - The goal test requires that the agent can stay at its goal forever:
///    t > latest vertex constraint on the goal and no permanent block.
///  - Beyond the last constrained time T_c the problem is time-invariant, so
///    states (v, t) with t > T_c are merged (duplicate detection on
///    (v, min(t, T_c + 1))); the first expansion has the smallest arrival time.
///  - The search horizon defaults to T_c + |V_free| + 1, after which a
///    shortest path in the static problem has surely been found; this
///    guarantees termination and completeness for the single-agent problem.

#include "multirobotcore/mapf/constraint_table.hpp"
#include "multirobotcore/mapf/grid_map.hpp"
#include "multirobotcore/mapf/instance.hpp"

#include <cstddef>
#include <optional>
#include <span>

namespace multirobotcore::mapf {

struct LowLevelStats {
  std::size_t expansions{0};  ///< nodes popped and expanded
  std::size_t generated{0};   ///< nodes pushed to the open list
};

struct LowLevelOptions {
  /// Maximum timestep considered; negative selects the automatic bound
  /// T_c + |V_free| + 1.
  int max_time{-1};
  /// Optional tie-breaking table (fewer conflicts with others first).
  const ConflictAvoidanceTable* conflict_avoidance{nullptr};
};

/// Shortest constrained path from start to goal (cost = arrival time), or
/// nullopt if none exists within the horizon. `heuristic` must be
/// map.distances_from(goal). Statistics are accumulated into `stats`.
[[nodiscard]] std::optional<Path> space_time_astar(const GridMap& map, Cell start, Cell goal,
                                                   const ConstraintTable& constraints,
                                                   std::span<const int> heuristic,
                                                   LowLevelStats& stats,
                                                   const LowLevelOptions& options = {});

}  // namespace multirobotcore::mapf
