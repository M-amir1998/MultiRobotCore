#include "multirobotcore/formation/safety.hpp"

#include <algorithm>
#include <stdexcept>

namespace multirobotcore::formation {

Vec2 separation_velocity(std::size_t i, std::span<const Vec2> positions, double safety_distance,
                         double gain) {
  if (i >= positions.size()) {
    throw std::out_of_range("separation_velocity: robot index out of range");
  }
  Vec2 u = Vec2::Zero();
  for (std::size_t j = 0; j < positions.size(); ++j) {
    if (j == i) {
      continue;
    }
    const Vec2 d = positions[i] - positions[j];
    const double dist = std::max(d.norm(), 1e-6);
    if (dist < safety_distance) {
      u += gain * (1.0 / dist - 1.0 / safety_distance) * d / dist;
    }
  }
  return u;
}

Vec2 obstacle_repulsion(const Vec2& position, const math::ObstacleSet& obstacles,
                        double robot_radius, double influence, double gain) {
  if (obstacles.empty() || !(influence > 0.0)) {
    return Vec2::Zero();
  }
  const double clearance = obstacles.distance(position) - robot_radius;
  if (clearance >= influence) {
    return Vec2::Zero();
  }
  const Vec2 away = position - obstacles.closest_point(position);
  const double n = away.norm();
  if (n < 1e-9) {
    return Vec2::Zero();  // inside an obstacle: no well-defined direction
  }
  const double c = std::max(clearance, 0.01);
  return gain * (1.0 / c - 1.0 / influence) * away / n;
}

}  // namespace multirobotcore::formation
