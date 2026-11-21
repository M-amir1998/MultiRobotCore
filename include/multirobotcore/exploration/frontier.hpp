#pragma once

/// @file frontier.hpp
/// Frontier detection, clustering and grid travel costs (Yamauchi 1997).
///
/// A frontier cell is a known-free cell with at least one unknown
/// 4-neighbour. Frontier cells are grouped into 8-connected clusters; each
/// cluster is represented by its member closest to the cluster centroid, which
/// is a known-free (hence plannable) cell even when the centroid is not.

#include "multirobotcore/exploration/occupancy_grid.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace multirobotcore::exploration {

inline constexpr int kUnreachable = -1;

[[nodiscard]] bool is_frontier(const OccupancyGrid& map, const GridCell& cell) noexcept;

/// All frontier cells in index order.
[[nodiscard]] std::vector<GridCell> detect_frontiers(const OccupancyGrid& map);

struct FrontierCluster {
  std::vector<GridCell> cells;  ///< sorted by index
  Vec2 centroid{Vec2::Zero()};  ///< mean cell coordinate (grid units)
  GridCell representative;      ///< member closest to the centroid
};

/// 8-connected clusters with at least `min_size` cells, ordered by their
/// smallest cell index.
[[nodiscard]] std::vector<FrontierCluster> cluster_frontiers(const OccupancyGrid& map,
                                                             std::span<const GridCell> frontiers,
                                                             std::size_t min_size = 1);

/// 4-connected BFS distances [cells] over known-free cells from `start`
/// (the start cell itself is always traversable). kUnreachable elsewhere.
[[nodiscard]] std::vector<int> bfs_distances(const OccupancyGrid& map, const GridCell& start);

/// Shortest 4-connected path over known-free cells, including both ends;
/// empty when the goal is unreachable.
[[nodiscard]] std::vector<GridCell> shortest_path(const OccupancyGrid& map, const GridCell& start,
                                                  const GridCell& goal);

/// Unknown cells within Euclidean distance `radius_cells` of `target`: an
/// optimistic estimate of the information gained by sensing from `target`.
[[nodiscard]] std::size_t information_gain(const OccupancyGrid& map, const GridCell& target,
                                           double radius_cells);

}  // namespace multirobotcore::exploration
