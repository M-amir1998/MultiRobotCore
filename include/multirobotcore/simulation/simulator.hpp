#pragma once

/// @file simulator.hpp
/// Lightweight deterministic multi-robot simulator.
///
/// The simulator exists to test, benchmark and visualise algorithms; it is not
/// a physics engine. Robots are discs. Each step the caller supplies one
/// desired planar velocity per robot (the common interface of the
/// coordination, formation, coverage and avoidance controllers); each robot's
/// dynamics model converts it into motion:
///
///  - single integrator: v = sat(v_des) (optionally acceleration-limited)
///  - double integrator: a = sat(k (v_des - v)), exact ZOH integration
///  - unicycle / differential drive: look-ahead point feedback linearisation
///
/// Collisions are detected, counted as contact events and reported, but not
/// resolved: overlapping robots pass through each other so that failures of a
/// coordination algorithm stay visible in the metrics.
///
/// Timing: one step advances continuous time by WorldConfig::dt [s]; time is
/// computed as step_count * dt to avoid accumulation drift.

#include "multirobotcore/math/geometry.hpp"
#include "multirobotcore/math/obstacles.hpp"
#include "multirobotcore/math/types.hpp"
#include "multirobotcore/models/robot_models.hpp"
#include "multirobotcore/models/robot_state.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <span>
#include <vector>

namespace multirobotcore::simulation {

enum class Dynamics : std::uint8_t {
  kSingleIntegrator,
  kDoubleIntegrator,
  kUnicycle,
  kDifferentialDrive,
};

struct RobotSpec {
  double radius{0.2};                                                ///< [m]
  double max_speed{1.0};                                             ///< [m/s]
  double max_acceleration{std::numeric_limits<double>::infinity()};  ///< [m/s^2]
  Dynamics dynamics{Dynamics::kSingleIntegrator};
  double velocity_gain{8.0};  ///< double integrator velocity-tracking gain [1/s]
  double lookahead{0.15};     ///< unicycle / differential drive look-ahead [m]
  double max_yaw_rate{4.0};   ///< [rad/s]
  models::DifferentialDriveParams drive{};
};

/// A robot is "stalled" when it is active but moved less than `min_progress`
/// metres over the last `window` seconds. This is a heuristic indicator of
/// deadlock/livelock, not a formal deadlock proof.
struct StallConfig {
  double window{4.0};         ///< [s]
  double min_progress{0.05};  ///< [m]
};

struct WorldConfig {
  double dt{0.05};                   ///< control and integration step [s]
  std::optional<math::AABB> bounds;  ///< workspace boundary
  math::ObstacleSet obstacles;       ///< static obstacles
  bool record{true};                 ///< store trajectory frames
  std::size_t record_every{1};       ///< record every n-th step
  StallConfig stall{};
};

struct Frame {
  double time{0.0};
  std::vector<Pose2> poses;
  std::vector<Vec2> velocities;
  std::vector<models::RobotStatus> status;
};

/// Start of a contact. `other` is empty for obstacle or boundary contacts.
struct CollisionEvent {
  double time{0.0};
  std::size_t robot{0};
  std::optional<std::size_t> other;
};

struct SimulationMetrics {
  std::size_t robot_collisions{0};     ///< robot-robot contact events
  std::size_t obstacle_collisions{0};  ///< robot-obstacle/boundary contact events
  /// Minimum over time and pairs of ||p_i - p_j|| - r_i - r_j [m]
  /// (negative while overlapping).
  double min_pair_clearance{std::numeric_limits<double>::infinity()};
  /// Minimum robot-surface-to-obstacle distance [m].
  double min_obstacle_clearance{std::numeric_limits<double>::infinity()};
  std::vector<double> distance_travelled;  ///< per robot [m]
  std::vector<double> control_effort;      ///< per robot, sum ||v_cmd||^2 dt [m^2/s]
};

class Simulator {
 public:
  Simulator(WorldConfig config, std::vector<RobotSpec> specs, std::vector<Pose2> initial_poses);

  [[nodiscard]] std::size_t num_robots() const noexcept { return robots_.size(); }
  [[nodiscard]] double dt() const noexcept { return config_.dt; }
  [[nodiscard]] double time() const noexcept {
    return static_cast<double>(step_count_) * config_.dt;
  }
  [[nodiscard]] std::size_t step_count() const noexcept { return step_count_; }

  [[nodiscard]] const std::vector<models::RobotState>& robots() const noexcept { return robots_; }
  [[nodiscard]] const models::RobotState& robot(std::size_t i) const { return robots_.at(i); }
  [[nodiscard]] const RobotSpec& spec(std::size_t i) const { return specs_.at(i); }
  [[nodiscard]] const std::vector<RobotSpec>& specs() const noexcept { return specs_; }
  [[nodiscard]] std::vector<Vec2> positions() const;
  [[nodiscard]] std::vector<Vec2> velocities() const;
  /// active[i] is true when robot i can move and communicate.
  [[nodiscard]] std::vector<bool> operational_mask() const;

  [[nodiscard]] const WorldConfig& config() const noexcept { return config_; }
  [[nodiscard]] const math::ObstacleSet& obstacles() const noexcept { return config_.obstacles; }

  void set_status(std::size_t i, models::RobotStatus status);
  void set_goal(std::size_t i, std::optional<Vec2> goal);
  void set_task(std::size_t i, std::optional<std::size_t> task);
  void set_neighbors(std::size_t i, std::vector<models::RobotId> neighbors);

  void add_obstacle(const math::Circle& circle);
  void add_obstacle(const math::AABB& box);

  /// Advances the world by one step. `desired_velocities` holds one entry per
  /// robot; entries for non-operational robots are ignored. Throws on size
  /// mismatch or non-finite commands.
  void step(std::span<const Vec2> desired_velocities);

  [[nodiscard]] const SimulationMetrics& metrics() const noexcept { return metrics_; }
  [[nodiscard]] const std::vector<CollisionEvent>& collision_events() const noexcept {
    return collision_events_;
  }
  [[nodiscard]] const std::vector<Frame>& frames() const noexcept { return frames_; }

  /// Active robots that made less than StallConfig::min_progress over the
  /// last StallConfig::window seconds.
  [[nodiscard]] std::vector<std::size_t> stalled_robots() const;
  /// True when there is at least one active robot and all active robots are
  /// stalled.
  [[nodiscard]] bool stalled() const;

 private:
  void integrate(std::size_t i, const Vec2& command);
  void enforce_bounds(std::size_t i);
  void detect_collisions();
  void record_frame();

  WorldConfig config_;
  std::vector<RobotSpec> specs_;
  std::vector<models::RobotState> robots_;
  std::size_t step_count_{0};

  SimulationMetrics metrics_;
  std::vector<CollisionEvent> collision_events_;
  std::vector<char> pair_contact_;      // upper-triangular contact flags
  std::vector<char> obstacle_contact_;  // per robot
  std::vector<char> boundary_hit_;      // per robot, set during the current step
  std::vector<Frame> frames_;
  std::deque<std::vector<Vec2>> history_;  // positions over the stall window
};

}  // namespace multirobotcore::simulation
