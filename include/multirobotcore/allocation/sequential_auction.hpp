#pragma once

/// @file sequential_auction.hpp
/// Multi-task allocation by Sequential Single-Item (SSI) auctions (Koenig et
/// al. 2006): tasks are allocated one per round; in every round each robot
/// with spare capacity bids its marginal cost of inserting each unallocated
/// task at the cheapest position of its current route, and the globally
/// lowest bid wins. SSI is a fast heuristic with known approximation
/// guarantees for the MiniSum objective in metric spaces; it is not an exact
/// multi-robot routing (vehicle-routing) solver.

#include "multirobotcore/allocation/types.hpp"

#include <cstddef>
#include <limits>
#include <span>
#include <vector>

namespace multirobotcore::allocation {

enum class TeamObjective {
  kMiniSum,  ///< minimise the sum of route lengths (total travel)
  kMiniMax,  ///< minimise the longest route (makespan proxy)
};

struct SequentialAuctionConfig {
  TeamObjective objective{TeamObjective::kMiniSum};
  std::size_t max_tasks_per_robot{std::numeric_limits<std::size_t>::max()};
};

struct RoutePlan {
  /// Ordered task indices per robot.
  std::vector<std::vector<std::size_t>> sequences;
  std::vector<double> route_length;      ///< per robot [m]
  double total_length{0.0};              ///< MiniSum value [m]
  double max_length{0.0};                ///< MiniMax value [m]
  std::vector<std::size_t> unallocated;  ///< tasks left when capacity ran out
  std::size_t rounds{0};
  std::size_t bids{0};
};

/// Length of a route that starts at `start` and visits `tasks` in order
/// (pickup, then drop-off when present).
[[nodiscard]] double route_length(const Vec2& start, std::span<const Task> tasks,
                                  std::span<const std::size_t> order);

[[nodiscard]] RoutePlan sequential_single_item_auction(std::span<const Robot> robots,
                                                       std::span<const Task> tasks,
                                                       const SequentialAuctionConfig& config = {});

}  // namespace multirobotcore::allocation
