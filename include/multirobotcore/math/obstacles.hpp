#pragma once

/// @file obstacles.hpp
/// Static obstacle set shared by the communication model (line of sight), the
/// simulator (collision detection) and local planners.

#include "multirobotcore/math/geometry.hpp"

#include <limits>
#include <vector>

namespace multirobotcore::math {

struct ObstacleSet {
  std::vector<Circle> circles;
  std::vector<AABB> boxes;

  [[nodiscard]] bool empty() const noexcept { return circles.empty() && boxes.empty(); }

  /// True when the segment touches any obstacle (line-of-sight test).
  [[nodiscard]] bool blocks(const Segment& segment) const noexcept;

  /// True when a disc-shaped body overlaps any obstacle.
  [[nodiscard]] bool collides(const Circle& body) const noexcept;

  /// Distance from p to the closest obstacle surface (0 inside an obstacle,
  /// +infinity when there are no obstacles).
  [[nodiscard]] double distance(const Vec2& p) const noexcept;

  /// Closest point on any obstacle to p (p itself when inside). Undefined
  /// (returns p) when the set is empty.
  [[nodiscard]] Vec2 closest_point(const Vec2& p) const noexcept;
};

}  // namespace multirobotcore::math
