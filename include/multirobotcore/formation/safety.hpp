#pragma once

/// @file safety.hpp
/// Short-range repulsive terms that keep formation controllers from driving
/// robots into each other or into obstacles. They are reactive safety
/// heuristics (barrier-like potentials), not a certified collision-avoidance
/// method; see the avoidance module for velocity-obstacle based methods.

#include "multirobotcore/math/obstacles.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <span>

namespace multirobotcore::formation {

/// sum_{j != i, d_ij < d_s} gain (1/d_ij - 1/d_s) (p_i - p_j) / d_ij
[[nodiscard]] Vec2 separation_velocity(std::size_t i, std::span<const Vec2> positions,
                                       double safety_distance, double gain);

/// gain (1/c - 1/c_0) n for obstacle clearance c = dist(p, O) - r < c_0, where
/// n points away from the closest obstacle point. Clearances below 1 cm are
/// clamped to keep the term finite.
[[nodiscard]] Vec2 obstacle_repulsion(const Vec2& position, const math::ObstacleSet& obstacles,
                                      double robot_radius, double influence, double gain);

}  // namespace multirobotcore::formation
