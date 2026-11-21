#pragma once

/// @file instance.hpp
/// MAPF problem instances, paths and solution costs.
///
/// A path lists one cell per timestep starting at t = 0. After its last entry
/// an agent is assumed to stay ("park") at that cell forever, which is the
/// standard sum-of-costs/makespan semantics of Stern et al. (2019).

#include "multirobotcore/mapf/grid_map.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace multirobotcore::mapf {

using Path = std::vector<Cell>;

struct Agent {
  Cell start;
  Cell goal;
};

struct Instance {
  GridMap map;
  std::vector<Agent> agents;
};

/// Throws std::invalid_argument when a start/goal is blocked or out of
/// bounds, or when two agents share a start or a goal.
void validate_instance(const Instance& instance);

/// Position of an agent at time t, accounting for parking after the path ends.
[[nodiscard]] Cell position_at(const Path& path, std::size_t t);

/// Arrival time: the first timestep from which the path stays at its final
/// cell. Trailing waits at the goal therefore cost nothing. Empty path -> 0.
[[nodiscard]] std::size_t path_cost(const Path& path);

/// Sum over agents of path_cost.
[[nodiscard]] std::size_t sum_of_costs(const std::vector<Path>& paths);

/// Maximum over agents of path_cost.
[[nodiscard]] std::size_t makespan(const std::vector<Path>& paths);

/// Removes redundant trailing waits at the final cell.
[[nodiscard]] Path trim_trailing_waits(Path path);

struct PathCheck {
  bool valid{true};
  std::string message;
};

/// Checks that every path starts at its start, ends at its goal, stays on free
/// cells, and only moves to 4-neighbours or waits. Collisions are checked
/// separately (conflicts.hpp).
[[nodiscard]] PathCheck check_paths(const Instance& instance, const std::vector<Path>& paths);

/// A timed waypoint in SI units.
struct TimedWaypoint {
  double time{0.0};             ///< [s]
  Vec2 position{Vec2::Zero()};  ///< [m]
};

/// Converts a grid path into one SI waypoint per timestep:
/// time = t * step_duration, position = frame.to_world(path[t]).
/// Used to hand MAPF plans to continuous controllers and simulators.
[[nodiscard]] std::vector<TimedWaypoint> to_waypoints(const Path& path, const WorldFrame& frame,
                                                      double step_duration);

}  // namespace multirobotcore::mapf
