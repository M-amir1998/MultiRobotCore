#pragma once

/// @file sensor.hpp
/// Deterministic 360-degree range sensor simulated on a ground-truth grid.
///
/// Each beam is traversed with the incremental grid-traversal algorithm of
/// Amanatides & Woo (1987): every cell the beam passes through is observed
/// (free), and a beam stops at the first occupied cell (observed occupied),
/// at the map border or at the maximum range. Noise-free by design: the
/// exploration benchmark isolates coordination effects.

#include "multirobotcore/exploration/occupancy_grid.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <span>
#include <vector>

namespace multirobotcore::exploration {

struct RangeSensorConfig {
  double range{4.0};   ///< maximum range [m]
  int num_beams{180};  ///< evenly spaced over 360 degrees, offset by half a step
};

/// Cells observed from `origin` (world coordinates), sorted by index, unique.
[[nodiscard]] std::vector<GridCell> simulate_scan(const OccupancyGrid& truth, const Vec2& origin,
                                                  const RangeSensorConfig& config);

/// Copies the true occupancy of `observed` cells into `belief`. Returns the
/// number of cells that were previously unknown.
std::size_t integrate_scan(OccupancyGrid& belief, const OccupancyGrid& truth,
                           std::span<const GridCell> observed);

}  // namespace multirobotcore::exploration
