#pragma once

/// @file types.hpp
/// Fundamental value types shared by every MultiRobotCore module.
///
/// Units: SI throughout (metres, seconds, radians, metres/second). Discrete
/// MAPF grids use their own integer cell/timestep units, documented in
/// multirobotcore/mapf.

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace multirobotcore {

/// Planar vector: a position [m], velocity [m/s] or acceleration [m/s^2].
using Vec2 = Eigen::Vector2d;

/// Ordered list of planar vectors, one entry per robot unless stated otherwise.
using Vec2List = std::vector<Vec2>;

/// Planar pose: position [m] and heading [rad] measured counter-clockwise from +x.
struct Pose2 {
  Vec2 position{Vec2::Zero()};
  double heading{0.0};
};

}  // namespace multirobotcore
