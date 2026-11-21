#pragma once

/// @file conflict_based_search.hpp
/// Conflict-Based Search (Sharon, Stern, Felner & Sturtevant 2015).
///
/// High level: best-first search over a constraint tree (CT). Each CT node
/// stores a set of constraints, one path per agent that is individually
/// shortest under the agent's constraints, the node cost, and its number of
/// conflicts. The node with the lowest cost is expanded (ties: fewer
/// conflicts, then generation order). Its earliest conflict <a1, a2, ...> is
/// resolved by branching into two children, each adding one constraint:
///   vertex conflict (v, t)  ->  <a1, v, t>  |  <a2, v, t>
///   edge conflict (u, v, t) ->  <a1, u->v, t> | <a2, v->u, t>
/// Only the newly constrained agent is replanned (space-time A*).
///
/// Conflict selection: by default cardinal conflicts are split first
/// (the conflict prioritisation of Improved CBS, Boyarski et al. 2015);
/// otherwise the earliest conflict is split.
///
/// Objectives: sum of costs (default) or makespan. For both, a CT node's cost
/// is a lower bound on the cost of every valid solution consistent with its
/// constraints, so the first conflict-free node expanded is optimal for the
/// chosen objective. Paths of unchanged agents are shared between nodes.

#include "multirobotcore/mapf/instance.hpp"
#include "multirobotcore/mapf/planners.hpp"

#include <cstddef>

namespace multirobotcore::mapf {

enum class Objective { kSumOfCosts, kMakespan };

struct CbsOptions {
  Objective objective{Objective::kSumOfCosts};
  std::size_t max_high_level_nodes{200000};  ///< expansion limit
  double time_limit_s{30.0};                 ///< wall-clock limit
  /// Break low-level ties towards fewer conflicts with the other agents'
  /// current paths (does not change optimality).
  bool use_conflict_avoidance{true};
  /// Split cardinal conflicts first, then semi-cardinal ones (Boyarski et al.
  /// 2015), using multi-valued decision diagrams (MDDs) of the agents'
  /// shortest constrained paths. This changes only which conflict is split,
  /// so optimality is preserved; it usually shrinks the constraint tree.
  bool prioritize_cardinal_conflicts{true};
};

class ConflictBasedSearch {
 public:
  explicit ConflictBasedSearch(CbsOptions options = {}) : options_(options) {}

  /// Solves the instance. Status kSolved with optimal paths, or kTimeout /
  /// kNodeLimit / kNoSolution. Throws std::invalid_argument for invalid
  /// instances.
  [[nodiscard]] PlannerResult solve(const Instance& instance) const;

  [[nodiscard]] const CbsOptions& options() const noexcept { return options_; }

 private:
  CbsOptions options_;
};

}  // namespace multirobotcore::mapf
