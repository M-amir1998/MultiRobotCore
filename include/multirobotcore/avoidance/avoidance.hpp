#pragma once

/// @file avoidance.hpp
/// Team-level local collision avoidance: maps preferred velocities of all
/// robots to safe velocities with VO, RVO or ORCA, using only neighbours
/// within a sensing radius.
///
/// Obstacles: discs are used directly; axis-aligned boxes are covered by a
/// conservative chain/ring of discs (obstacle_discs()). Robots that are not
/// active (stopped or failed) are treated by the others as static,
/// non-reciprocating obstacles and receive a zero velocity.

#include "multirobotcore/avoidance/orca.hpp"
#include "multirobotcore/avoidance/sampling.hpp"
#include "multirobotcore/avoidance/velocity_obstacle.hpp"
#include "multirobotcore/math/geometry.hpp"
#include "multirobotcore/math/obstacles.hpp"
#include "multirobotcore/math/types.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace multirobotcore::avoidance {

enum class AvoidanceMethod {
  kNone,                        ///< pass preferred velocities through (unsafe baseline)
  kVelocityObstacle,            ///< sampling-based VO
  kReciprocalVelocityObstacle,  ///< sampling-based RVO
  kOrca,                        ///< ORCA half-planes + linear programming
};

[[nodiscard]] std::string_view to_string(AvoidanceMethod method) noexcept;
/// Parses "none", "vo", "rvo" or "orca". Throws std::invalid_argument.
[[nodiscard]] AvoidanceMethod parse_method(std::string_view name);

/// Defaults follow common practice for ground robots moving at ~1 m/s. A
/// small safety margin absorbs the residual overlap that ORCA can produce when
/// its linear program is infeasible (dense crowds) or when neighbour sets are
/// not mutual because of the max_neighbors cap.
struct AvoidanceConfig {
  AvoidanceMethod method{AvoidanceMethod::kOrca};
  double time_horizon{3.0};            ///< tau for robots [s]
  double obstacle_time_horizon{1.5};   ///< tau for static obstacles [s]
  double time_step{0.05};              ///< control period [s]
  double neighbor_distance{5.0};       ///< sensing radius (centre to centre) [m]
  std::size_t max_neighbors{16};       ///< closest robots considered
  double safety_margin{0.02};          ///< added to every robot radius [m]
  double obstacle_disc_spacing{0.25};  ///< box discretisation step [m]
  std::size_t speed_rings{6};          ///< sampling methods
  std::size_t angle_samples{36};       ///< sampling methods
  double penalty_weight{1.0};          ///< sampling methods [m]
};

/// Covers the obstacle set with discs: circles are kept, every box is covered
/// by the boundary cells of a grid with cells of at most `max_spacing` (one
/// disc circumscribing each cell). Conservative by at most half a cell
/// diagonal.
[[nodiscard]] std::vector<math::Circle> obstacle_discs(const math::ObstacleSet& obstacles,
                                                       double max_spacing);

/// Velocity towards a goal at max_speed, decreasing linearly inside
/// `slowdown_distance` (arrival behaviour).
[[nodiscard]] Vec2 preferred_velocity(const Vec2& position, const Vec2& goal, double max_speed,
                                      double slowdown_distance = 0.5) noexcept;

/// Velocity-space diagnostics of one robot, for visualisation and analysis.
struct NeighborDiagnostics {
  std::size_t index{0};      ///< robot index if is_robot, otherwise obstacle-disc index
  bool is_robot{true};       ///< false for obstacle discs
  bool is_obstacle{false};   ///< non-reciprocating: obstacle disc or inactive robot
  VelocityObstacleCone vo;   ///< VO^tau
  VelocityObstacleCone rvo;  ///< RVO (agents only; equals vo for obstacles)
  OrcaConstraint orca;
};

struct AgentDiagnostics {
  Vec2 preferred{Vec2::Zero()};
  Vec2 selected{Vec2::Zero()};
  bool feasible{true};
  std::vector<NeighborDiagnostics> neighbors;
};

class CollisionAvoider {
 public:
  explicit CollisionAvoider(AvoidanceConfig config, const math::ObstacleSet& obstacles = {});

  /// Rebuilds the obstacle discs (e.g. after an obstacle appears).
  void set_obstacles(const math::ObstacleSet& obstacles);

  /// Safe velocities for all robots. `active` may be empty (all active).
  /// Throws std::invalid_argument on size mismatches.
  [[nodiscard]] std::vector<Vec2> compute(std::span<const AgentState> agents,
                                          std::span<const Vec2> preferred,
                                          std::span<const double> max_speeds,
                                          const std::vector<bool>& active = {}) const;

  /// Diagnostics for robot i under the configured method.
  [[nodiscard]] AgentDiagnostics diagnose(std::size_t i, std::span<const AgentState> agents,
                                          std::span<const Vec2> preferred,
                                          std::span<const double> max_speeds,
                                          const std::vector<bool>& active = {}) const;

  [[nodiscard]] const AvoidanceConfig& config() const noexcept { return config_; }
  [[nodiscard]] const std::vector<math::Circle>& discs() const noexcept { return discs_; }

  /// Number of robots whose ORCA program was infeasible in the last compute()
  /// call (always 0 for the other methods).
  [[nodiscard]] std::size_t last_infeasible_count() const noexcept { return last_infeasible_; }

 private:
  struct Neighborhood {
    std::vector<AgentState> agents;  ///< reciprocating robots
    std::vector<std::size_t> agent_ids;
    std::vector<AgentState> obstacles;  ///< discs and inactive robots
    std::vector<std::size_t> obstacle_ids;
    std::vector<bool> obstacle_is_robot;
  };

  [[nodiscard]] Neighborhood neighborhood(std::size_t i, std::span<const AgentState> agents,
                                          const std::vector<bool>& active) const;
  [[nodiscard]] Vec2 select(const AgentState& self, const Vec2& preferred, double max_speed,
                            const Neighborhood& hood, bool* feasible) const;

  AvoidanceConfig config_;
  std::vector<math::Circle> discs_;
  mutable std::size_t last_infeasible_{0};
};

}  // namespace multirobotcore::avoidance
