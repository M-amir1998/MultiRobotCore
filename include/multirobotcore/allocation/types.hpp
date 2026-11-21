#pragma once

/// @file types.hpp
/// Task-allocation domain objects: robots, tasks, cost matrices and
/// assignments (single-task robots, single-robot tasks, instantaneous
/// assignment: ST-SR-IA in the taxonomy of Gerkey & Mataric 2004).

#include "multirobotcore/math/types.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace multirobotcore::allocation {

/// Cost matrix: rows are robots, columns are tasks. +infinity marks an
/// infeasible robot-task pair; NaN is rejected by every solver.
using CostMatrix = Eigen::MatrixXd;

inline constexpr double kInfeasible = std::numeric_limits<double>::infinity();

struct Robot {
  std::size_t id{0};
  Vec2 position{Vec2::Zero()};   ///< [m]
  double speed{1.0};             ///< nominal travel speed [m/s]
  double energy_per_meter{1.0};  ///< [J/m]
  double capacity{1.0};          ///< payload capacity (task demand must fit)
};

struct Task {
  std::size_t id{0};
  Vec2 position{Vec2::Zero()};  ///< pickup / service location [m]
  std::optional<Vec2> dropoff;  ///< optional delivery location [m]
  double service_time{0.0};     ///< [s]
  double demand{0.0};           ///< required capacity
  double priority{1.0};         ///< larger is more important
};

/// Result of an assignment solver. At most one task per robot and one robot
/// per task.
struct Assignment {
  std::vector<std::optional<std::size_t>> robot_to_task;  ///< per row
  std::vector<std::optional<std::size_t>> task_to_robot;  ///< per column
  double total_cost{0.0};

  [[nodiscard]] std::size_t num_assigned() const noexcept;
  /// True when the two maps are mutually consistent and injective.
  [[nodiscard]] bool is_consistent() const noexcept;
};

/// Builds an Assignment from a row -> column map and evaluates its cost.
[[nodiscard]] Assignment make_assignment(
    const CostMatrix& costs, const std::vector<std::optional<std::size_t>>& robot_to_task);

/// Sum of costs of the assigned pairs.
[[nodiscard]] double assignment_cost(const CostMatrix& costs, const Assignment& assignment);

// --- Cost models ------------------------------------------------------------------

/// Straight-line distance robot -> task (+ task -> dropoff).
[[nodiscard]] CostMatrix euclidean_costs(std::span<const Robot> robots,
                                         std::span<const Task> tasks);

/// Travel-time estimate: distance / speed + service time [s].
[[nodiscard]] CostMatrix travel_time_costs(std::span<const Robot> robots,
                                           std::span<const Task> tasks);

/// Energy estimate: energy_per_meter * distance [J].
[[nodiscard]] CostMatrix energy_costs(std::span<const Robot> robots, std::span<const Task> tasks);

struct CostWeights {
  double distance{1.0};         ///< per metre
  double time{0.0};             ///< per second
  double energy{0.0};           ///< per joule
  double priority{0.0};         ///< subtracted per unit of task priority
  bool enforce_capacity{true};  ///< demand > capacity -> infeasible
};

/// Weighted combination of the models above.
[[nodiscard]] CostMatrix weighted_costs(std::span<const Robot> robots, std::span<const Task> tasks,
                                        const CostWeights& weights);

/// Arbitrary user-defined cost.
[[nodiscard]] CostMatrix custom_costs(std::span<const Robot> robots, std::span<const Task> tasks,
                                      const std::function<double(const Robot&, const Task&)>& cost);

/// Throws std::invalid_argument when the matrix contains NaN or -infinity.
void validate_costs(const CostMatrix& costs);

}  // namespace multirobotcore::allocation
