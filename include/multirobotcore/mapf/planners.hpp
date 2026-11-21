#pragma once

/// @file planners.hpp
/// Coupled-free MAPF planners and the shared result type:
///
///  - independent A*: every agent ignores the others. Fast but unsafe; used
///    as the baseline that shows why coordination matters.
///  - Prioritized Planning (Erdmann & Lozano-Perez 1987; Silver 2005): agents
///    plan one at a time in priority order with space-time A*, treating the
///    paths of higher-priority agents as moving obstacles (reservation
///    table). Fast, but incomplete and suboptimal; the result depends on the
///    ordering, and it can fail on solvable instances.

#include "multirobotcore/mapf/instance.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace multirobotcore::mapf {

enum class PlannerStatus {
  kSolved,      ///< collision-free paths for every agent
  kUnsafe,      ///< paths found but they conflict (independent planning)
  kFailed,      ///< an agent could not be planned (prioritized planning)
  kNoSolution,  ///< search space exhausted
  kTimeout,     ///< time limit reached
  kNodeLimit,   ///< high-level node limit reached
};

[[nodiscard]] std::string_view to_string(PlannerStatus status) noexcept;

struct PlannerResult {
  PlannerStatus status{PlannerStatus::kNoSolution};
  std::vector<Path> paths;  ///< one per agent when status is kSolved/kUnsafe
  std::size_t sum_of_costs{0};
  std::size_t makespan{0};
  std::size_t conflicts{0};  ///< pairwise conflicts in `paths`
  double runtime_ms{0.0};
  std::size_t low_level_expansions{0};
  std::size_t high_level_expanded{0};   ///< CBS constraint-tree nodes expanded
  std::size_t high_level_generated{0};  ///< CBS constraint-tree nodes generated
  std::size_t attempts{0};              ///< prioritized planning orderings tried
  std::vector<std::size_t> order;       ///< prioritized planning order used
  std::string message;

  [[nodiscard]] bool solved() const noexcept { return status == PlannerStatus::kSolved; }
};

/// Individually optimal paths, ignoring the other agents.
[[nodiscard]] PlannerResult plan_independent(const Instance& instance);

enum class PriorityOrder {
  kIndex,          ///< agent 0 first
  kShortestFirst,  ///< ascending start-goal distance (ties by index)
  kLongestFirst,   ///< descending start-goal distance (ties by index)
  kRandom,         ///< seeded shuffle
  kCustom,         ///< PrioritizedOptions::custom_order
};

struct PrioritizedOptions {
  PriorityOrder order{PriorityOrder::kShortestFirst};
  std::vector<std::size_t> custom_order;
  std::uint64_t seed{0};
  /// Orderings to try; attempts after the first use seeded random orders.
  std::size_t max_attempts{1};
};

[[nodiscard]] PlannerResult plan_prioritized(const Instance& instance,
                                             const PrioritizedOptions& options = {});

}  // namespace multirobotcore::mapf
