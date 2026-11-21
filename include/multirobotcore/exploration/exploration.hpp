#pragma once

/// @file exploration.hpp
/// Discrete-time multi-robot frontier exploration with a shared map.
///
/// Time is discrete: in one step every robot moves at most one 4-connected
/// cell along its planned path and then scans (one step lasts resolution /
/// speed seconds for robots of speed `speed`). Robots are points on the grid;
/// inter-robot collisions are not modelled in this module (several robots may
/// occupy one cell): the module studies the coordination of exploration
/// targets, while collision-free motion is handled by the MAPF and avoidance
/// modules.
///
/// Communication model (centralised map server): every robot that observed
/// new cells uploads them once per step (message_bytes_per_cell bytes each),
/// and the server sends a target to every robot after each re-planning.

#include "multirobotcore/exploration/frontier.hpp"
#include "multirobotcore/exploration/occupancy_grid.hpp"
#include "multirobotcore/exploration/planner.hpp"
#include "multirobotcore/exploration/sensor.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace multirobotcore::exploration {

struct ExplorationConfig {
  PlannerConfig planner{};
  RangeSensorConfig sensor{};
  std::size_t max_steps{3000};
  std::size_t replan_period{10};          ///< steps between periodic re-planning
  double completion_fraction{1.0};        ///< stop at this reachable-free-space coverage
  std::size_t message_bytes_per_cell{5};  ///< uint32 index + occupancy byte
  bool record_frames{false};
};

struct ExplorationFrame {
  std::size_t step{0};
  std::vector<GridCell> robots;
  std::vector<std::optional<GridCell>> targets;
  std::vector<GridCell> newly_known;  ///< cells first observed this step
  std::vector<GridCell> frontier;     ///< frontier cells after this step's scans
  double coverage{0.0};
};

struct ExplorationResult {
  std::vector<double> coverage;              ///< per step, index 0 = after the initial scans
  std::vector<std::vector<GridCell>> paths;  ///< executed path per robot
  std::size_t steps{0};
  bool complete{false};  ///< no reachable frontier left or target coverage reached
  std::optional<std::size_t> steps_to_90;
  std::optional<std::size_t> steps_to_95;
  std::vector<double> distance;    ///< per robot [m]
  std::size_t observed_cells{0};   ///< cells observed by at least one robot
  std::size_t redundant_cells{0};  ///< cells observed by two or more robots
  std::size_t replans{0};
  std::size_t messages{0};
  std::size_t bytes{0};
  std::vector<ExplorationFrame> frames;  ///< when record_frames
};

class ExplorationSimulation {
 public:
  ExplorationSimulation(OccupancyGrid truth, std::vector<GridCell> starts,
                        ExplorationConfig config);

  /// Advances one step; returns false once exploration has finished.
  bool step();

  /// Runs until completion or max_steps and returns the result.
  ExplorationResult run();

  [[nodiscard]] const OccupancyGrid& belief() const noexcept { return belief_; }
  [[nodiscard]] const OccupancyGrid& truth() const noexcept { return truth_; }
  [[nodiscard]] const std::vector<GridCell>& robots() const noexcept { return robots_; }
  [[nodiscard]] double coverage() const noexcept;
  [[nodiscard]] bool finished() const noexcept { return finished_; }
  [[nodiscard]] const ExplorationResult& result() const noexcept { return result_; }
  /// Number of free cells reachable from the start positions (ground truth).
  [[nodiscard]] std::size_t reachable_free_cells() const noexcept { return reachable_free_; }

 private:
  void scan_all(std::vector<GridCell>& newly_known);
  void replan();
  void record(const std::vector<GridCell>& newly_known);
  [[nodiscard]] bool frontier_reachable() const;

  OccupancyGrid truth_;
  OccupancyGrid belief_;
  ExplorationConfig config_;
  std::vector<GridCell> robots_;
  std::vector<std::optional<GridCell>> targets_;
  std::vector<std::vector<GridCell>> plans_;  // remaining path per robot (next cell first)
  std::vector<char> reachable_mask_;          // ground-truth reachable free cells
  std::vector<std::uint64_t> observers_;      // bit i set when robot i observed the cell
  std::size_t reachable_free_{0};
  std::size_t known_reachable_{0};
  std::size_t step_{0};
  std::size_t since_replan_{0};
  bool finished_{false};
  ExplorationResult result_;
};

}  // namespace multirobotcore::exploration
