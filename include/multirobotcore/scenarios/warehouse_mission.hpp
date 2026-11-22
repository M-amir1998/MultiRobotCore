#pragma once

/// @file warehouse_mission.hpp
/// Integrated warehouse mission: the full MultiRobotCore pipeline
///
///   task requests -> task allocation (Hungarian, grid travel costs)
///   -> multi-agent path finding (prioritized planning, CBS fallback)
///   -> timed waypoints -> trajectory tracking
///   -> local reciprocal collision avoidance (ORCA) -> simulator
///
/// Robots wait at home cells on the right edge, fetch items at shelf access
/// points and deliver them to packing stations on the left edge. Home cells,
/// pickups and stations are disjoint, so MAPF goals never coincide. Planning
/// happens at MAPF step boundaries whenever the situation changes (assignment,
/// arrival at a goal, end of a service); every robot is re-planned from the
/// cell it occupies in the current plan, which keeps consecutive plans
/// consistent and collision-free in the discrete model. If no plan is found
/// the previous collision-free plan is kept and planning is retried at the
/// next step.
///
/// Continuous execution can deviate from the discrete plan (acceleration
/// limits, ORCA adjustments). The tracking controller pulls robots back onto
/// the timed reference, and ORCA guarantees separation only under its own
/// assumptions; contacts and the largest tracking error are reported so that
/// such deviations stay visible.

#include "multirobotcore/io/json.hpp"
#include "multirobotcore/mapf/generators.hpp"
#include "multirobotcore/mapf/grid_map.hpp"
#include "multirobotcore/mapf/instance.hpp"
#include "multirobotcore/math/obstacles.hpp"
#include "multirobotcore/scenarios/formation_mission.hpp"
#include "multirobotcore/simulation/simulator.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace multirobotcore::scenarios {

struct WarehouseMissionConfig {
  mapf::WarehouseLayout layout{};
  std::size_t num_robots{14};
  double cell_size{1.0};      ///< [m]
  double step_duration{1.0};  ///< duration of one MAPF timestep [s]
  double dt{0.05};            ///< simulation / control step [s]
  double duration{120.0};     ///< [s]
  std::uint64_t seed{7};
  std::size_t initial_tasks{10};
  double arrival_rate{0.25};           ///< task requests per second after t = 0
  double arrival_horizon{90.0};        ///< no new requests after this time [s]
  std::size_t service_steps{2};        ///< MAPF steps spent picking / dropping
  double robot_radius{0.3};            ///< [m]
  double max_speed{1.3};               ///< [m/s]
  double max_acceleration{4.0};        ///< [m/s^2]
  double tracking_gain{2.5};           ///< [1/s]
  bool local_avoidance{true};          ///< ORCA safety layer
  bool avoid_shelves{true};            ///< include shelves as ORCA static obstacles
  double orca_time_horizon{1.0};       ///< [s]
  std::string planner{"prioritized"};  ///< "prioritized" or "cbs"
  std::size_t prioritized_attempts{20};
  double cbs_time_limit{1.0};  ///< [s] (also used as fallback)
  std::size_t record_every{4};
  std::size_t preview_steps{6};  ///< planned cells recorded per robot and frame
};

enum class WarehouseRobotState : std::uint8_t {
  kIdle,  ///< no task (at or returning to its home cell)
  kToPickup,
  kPicking,
  kToDropoff,  ///< carrying an item
  kDropping,
};

[[nodiscard]] std::string_view to_string(WarehouseRobotState state) noexcept;

struct WarehouseTask {
  std::size_t id{0};
  mapf::Cell pickup;
  std::optional<mapf::Cell> dropoff;
  double arrival_time{0.0};
  double assigned_time{-1.0};
  double picked_time{-1.0};
  double delivered_time{-1.0};
  std::optional<std::size_t> robot;
};

struct WarehouseMissionResult {
  mapf::Warehouse warehouse;
  mapf::WorldFrame frame;
  std::vector<mapf::Cell> homes;
  std::vector<mapf::Cell> stations;  ///< drop-off stations
  std::vector<simulation::Frame> frames;
  std::vector<std::vector<WarehouseRobotState>> robot_states;  ///< per recorded frame
  std::vector<std::vector<std::vector<Vec2>>> routes;   ///< per frame, per robot: planned positions
  std::vector<std::vector<std::size_t>> visible_tasks;  ///< per frame: arrived, undelivered tasks
  std::vector<std::vector<std::optional<std::size_t>>> robot_tasks;  ///< per frame
  std::vector<WarehouseTask> tasks;
  std::vector<MissionEvent> events;
  simulation::SimulationMetrics metrics;

  std::size_t tasks_completed{0};
  double mean_completion_time{0.0};  ///< arrival -> delivery [s]
  double throughput_per_minute{0.0};
  std::size_t replans{0};
  std::size_t planning_failures{0};  ///< epochs where the previous plan was kept
  std::size_t cbs_fallbacks{0};
  double mean_planning_ms{0.0};
  double max_planning_ms{0.0};
  std::size_t plan_conflicts{0};   ///< conflicts found in executed plans (expected 0)
  double max_tracking_error{0.0};  ///< [m]
  std::size_t orca_infeasible{0};  ///< ORCA programs that needed the fallback
};

struct CoordinatorStats {
  std::size_t tasks_completed{0};
  std::size_t replans{0};
  std::size_t planning_failures{0};
  std::size_t cbs_fallbacks{0};
  double total_planning_ms{0.0};
  double max_planning_ms{0.0};
  std::size_t plan_conflicts{0};
  double max_tracking_error{0.0};
};

/// The algorithmic core of the warehouse mission, independent of any
/// simulator or middleware: owns the task queue, allocation, MAPF plans and
/// trajectory tracking. Call update() every control step with the current
/// time and measured robot positions; it returns preferred velocities to be
/// passed through a safety layer (ORCA) to the robots. Used by
/// run_warehouse_mission() with the built-in simulator and by the ROS 2
/// coordinator node with any robot backend.
class WarehouseCoordinator {
 public:
  explicit WarehouseCoordinator(const WarehouseMissionConfig& config);

  [[nodiscard]] const WarehouseMissionConfig& config() const noexcept { return config_; }
  [[nodiscard]] const mapf::Warehouse& warehouse() const noexcept { return warehouse_; }
  [[nodiscard]] const mapf::WorldFrame& frame() const noexcept { return frame_; }
  [[nodiscard]] const std::vector<mapf::Cell>& homes() const noexcept { return homes_; }
  [[nodiscard]] const std::vector<mapf::Cell>& stations() const noexcept { return stations_; }
  [[nodiscard]] std::size_t num_robots() const noexcept { return robots_.size(); }

  /// Initial robot poses (home cells, facing -x).
  [[nodiscard]] std::vector<Pose2> home_poses() const;
  /// Shelves as axis-aligned boxes [m].
  [[nodiscard]] math::ObstacleSet shelf_obstacles() const;

  /// Adds a pick-and-deliver request. With `near`, the free shelf access point
  /// closest to that position is used; otherwise a random free one. Returns
  /// the task id, or nullopt when every pickup already has an open task.
  std::optional<std::size_t> request_task(double time, std::optional<Vec2> near = std::nullopt);

  /// Advances the coordinator to `time` [s] given measured positions [m] and
  /// returns one preferred velocity per robot [m/s]. Allocation and planning
  /// run when `time` crosses a MAPF step boundary.
  [[nodiscard]] std::vector<Vec2> update(double time, std::span<const Vec2> positions);

  [[nodiscard]] const std::vector<WarehouseTask>& tasks() const noexcept { return tasks_; }
  [[nodiscard]] const std::vector<MissionEvent>& events() const noexcept { return events_; }
  [[nodiscard]] std::vector<WarehouseRobotState> robot_states() const;
  [[nodiscard]] std::vector<std::optional<std::size_t>> robot_task_ids() const;
  /// Current position followed by the next `steps` planned cell centres [m].
  [[nodiscard]] std::vector<Vec2> planned_route(std::size_t robot, const Vec2& position,
                                                std::size_t steps) const;
  /// World positions of a robot's full current plan with timestamps [s].
  [[nodiscard]] std::vector<std::pair<double, Vec2>> timed_plan(std::size_t robot) const;
  [[nodiscard]] const CoordinatorStats& stats() const noexcept { return stats_; }

 private:
  struct Robot {
    WarehouseRobotState state{WarehouseRobotState::kIdle};
    mapf::Cell home;
    mapf::Cell goal;
    std::optional<std::size_t> task;
    std::size_t service_until{0};
  };

  [[nodiscard]] mapf::Cell planned_cell(std::size_t robot, std::size_t step) const;
  [[nodiscard]] double grid_distance(mapf::Cell from, mapf::Cell to);
  /// Allocation, goal bookkeeping and re-planning at MAPF step k (time t).
  void process_step(std::size_t k, double t, std::span<const Vec2> positions);
  bool replan(std::size_t step, double time);

  WarehouseMissionConfig config_;
  mapf::Warehouse warehouse_;
  mapf::WorldFrame frame_;
  std::vector<mapf::Cell> homes_;
  std::vector<mapf::Cell> stations_;
  std::vector<Robot> robots_;
  std::vector<mapf::Path> plan_;
  std::size_t plan_start_{0};
  std::optional<std::size_t> last_step_;
  bool dirty_{true};
  std::vector<WarehouseTask> tasks_;
  std::vector<MissionEvent> events_;
  CoordinatorStats stats_;
  std::mt19937_64 rng_;
  std::vector<std::pair<int, std::vector<int>>> distance_cache_;
};

[[nodiscard]] WarehouseMissionResult run_warehouse_mission(const WarehouseMissionConfig& cfg);

[[nodiscard]] WarehouseMissionConfig warehouse_mission_config_from_json(const io::Json& json);

[[nodiscard]] io::Json to_json(const WarehouseMissionResult& result);

}  // namespace multirobotcore::scenarios
