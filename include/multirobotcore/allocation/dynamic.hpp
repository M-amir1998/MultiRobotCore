#pragma once

/// @file dynamic.hpp
/// Dynamic task allocation: tasks arrive, are cancelled or completed during
/// execution, and the allocation is re-solved when the task set changes.
///
/// Re-planning is an assignment with inertia: robots that are within
/// `commit_distance` of their task keep it, and switching a robot away from
/// its current task costs `switching_penalty`, which suppresses reassignment
/// for marginal gains (hysteresis against thrashing). With a zero penalty and
/// zero commit distance every update is a fresh optimal assignment.

#include "multirobotcore/allocation/auction.hpp"
#include "multirobotcore/allocation/types.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace multirobotcore::allocation {

enum class AllocationMethod { kHungarian, kAuction };

struct DynamicAllocatorConfig {
  AllocationMethod method{AllocationMethod::kHungarian};
  CostWeights weights{};
  double switching_penalty{0.0};  ///< cost units added when a robot changes task
  double commit_distance{0.5};    ///< [m]
  AuctionConfig auction{};
};

struct AllocationUpdate {
  /// Task id per robot (nullopt: idle).
  std::vector<std::optional<std::size_t>> robot_task;
  std::size_t reassigned{0};      ///< robots moved from one task to another
  std::size_t newly_assigned{0};  ///< idle robots that received a task
  std::size_t released{0};        ///< robots whose task disappeared
  double cost{0.0};               ///< total cost of the new assignment
  std::size_t messages{0};        ///< auction messages (0 for Hungarian)
};

class DynamicTaskAllocator {
 public:
  explicit DynamicTaskAllocator(DynamicAllocatorConfig config = {});

  /// Adds a task and returns its id (ids are unique and never reused).
  std::size_t add_task(Task task);
  /// Removes an open task. Returns false when the id is unknown.
  bool cancel_task(std::size_t id);
  /// Marks a task as completed (removed from the open set).
  bool complete_task(std::size_t id);

  [[nodiscard]] const std::vector<Task>& open_tasks() const noexcept { return tasks_; }
  [[nodiscard]] std::optional<Task> find_task(std::size_t id) const;

  /// Re-plans for the given robots. `current[i]` is robot i's current task id.
  [[nodiscard]] AllocationUpdate update(std::span<const Robot> robots,
                                        std::span<const std::optional<std::size_t>> current);

  [[nodiscard]] const DynamicAllocatorConfig& config() const noexcept { return config_; }

 private:
  DynamicAllocatorConfig config_;
  std::vector<Task> tasks_;
  std::size_t next_id_{0};
};

}  // namespace multirobotcore::allocation
