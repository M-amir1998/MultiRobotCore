#pragma once

/// @file planner.hpp
/// Frontier-to-robot assignment strategies for a shared (centralised) map.
///
///  - kNearestFrontier (Yamauchi 1998): every robot independently drives to
///    its closest reachable frontier. No coordination; robots may duplicate.
///  - kHungarian: optimal one-to-one assignment minimising
///        c(i, t) = lambda * d(i, t) - w_I * I(t)
///    with BFS distance d and information gain I (allocation::solve_hungarian).
///    Robots left over when frontiers are scarce take their nearest frontier.
///  - kUtilityDiscount (Burgard, Moors, Stachniss & Schneider 2005): greedy
///    selection of the (robot, target) pair maximising U_t - beta * V_it,
///    where V_it is the travel cost normalised by the largest reachable cost
///    and U_t starts as the normalised information gain; after each choice
///    the utility of every target t' is reduced by
///        P(||t - t'||) = max(0, 1 - ||t - t'|| / r)
///    so that targets likely covered by the chosen robot's sensor become less
///    attractive to the others.

#include "multirobotcore/exploration/frontier.hpp"
#include "multirobotcore/exploration/occupancy_grid.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace multirobotcore::exploration {

enum class Strategy : std::uint8_t { kNearestFrontier, kHungarian, kUtilityDiscount };

[[nodiscard]] std::string_view to_string(Strategy strategy) noexcept;
/// Parses "nearest_frontier", "hungarian" or "utility_discount".
[[nodiscard]] std::optional<Strategy> strategy_from_string(std::string_view name) noexcept;

struct PlannerConfig {
  Strategy strategy{Strategy::kHungarian};
  double information_weight{1.0};   ///< w_I (Hungarian)
  double cost_weight{1.0};          ///< lambda per cell (Hungarian), beta (utility discount)
  double discount_radius{6.0};      ///< r [cells] (utility discount), typically the sensor range
  std::size_t min_cluster_size{2};  ///< clusters smaller than this are ignored (falls back to 1)
};

struct TargetAssignment {
  std::vector<std::optional<GridCell>> targets;      ///< per robot
  std::vector<std::optional<std::size_t>> clusters;  ///< per robot, index into the clusters
};

/// Assigns frontier clusters (via their representatives) to robots.
/// `sensor_radius_cells` is used for the information-gain estimate.
[[nodiscard]] TargetAssignment assign_targets(const OccupancyGrid& map,
                                              std::span<const GridCell> robots,
                                              std::span<const FrontierCluster> clusters,
                                              const PlannerConfig& config,
                                              double sensor_radius_cells);

}  // namespace multirobotcore::exploration
