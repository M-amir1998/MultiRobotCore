#include "multirobotcore/allocation/dynamic.hpp"

#include "multirobotcore/allocation/hungarian.hpp"

#include <algorithm>
#include <stdexcept>

namespace multirobotcore::allocation {

DynamicTaskAllocator::DynamicTaskAllocator(DynamicAllocatorConfig config) : config_(config) {}

std::size_t DynamicTaskAllocator::add_task(Task task) {
  task.id = next_id_++;
  tasks_.push_back(task);
  return task.id;
}

bool DynamicTaskAllocator::cancel_task(std::size_t id) {
  const auto it =
      std::find_if(tasks_.begin(), tasks_.end(), [id](const Task& t) { return t.id == id; });
  if (it == tasks_.end()) {
    return false;
  }
  tasks_.erase(it);
  return true;
}

bool DynamicTaskAllocator::complete_task(std::size_t id) {
  return cancel_task(id);
}

std::optional<Task> DynamicTaskAllocator::find_task(std::size_t id) const {
  const auto it =
      std::find_if(tasks_.begin(), tasks_.end(), [id](const Task& t) { return t.id == id; });
  if (it == tasks_.end()) {
    return std::nullopt;
  }
  return *it;
}

AllocationUpdate DynamicTaskAllocator::update(std::span<const Robot> robots,
                                              std::span<const std::optional<std::size_t>> current) {
  if (robots.size() != current.size()) {
    throw std::invalid_argument(
        "DynamicTaskAllocator::update: one current task per robot required");
  }
  AllocationUpdate out;
  out.robot_task.assign(robots.size(), std::nullopt);

  // Validate current tasks and keep committed robots on their tasks.
  std::vector<std::optional<std::size_t>> held(robots.size(), std::nullopt);
  std::vector<bool> task_taken(tasks_.size(), false);
  std::vector<std::size_t> free_robots;
  for (std::size_t i = 0; i < robots.size(); ++i) {
    std::optional<std::size_t> index;
    if (current[i]) {
      for (std::size_t k = 0; k < tasks_.size(); ++k) {
        if (tasks_[k].id == *current[i]) {
          index = k;
        }
      }
      if (!index) {
        ++out.released;
      }
    }
    held[i] = index;
    if (index && (tasks_[*index].position - robots[i].position).norm() <= config_.commit_distance) {
      out.robot_task[i] = tasks_[*index].id;
      task_taken[*index] = true;
    } else {
      free_robots.push_back(i);
    }
  }
  std::vector<std::size_t> open;
  for (std::size_t k = 0; k < tasks_.size(); ++k) {
    if (!task_taken[k]) {
      open.push_back(k);
    }
  }

  // Assignment with inertia over the free robots and open tasks.
  std::vector<Robot> sub_robots;
  sub_robots.reserve(free_robots.size());
  for (const std::size_t i : free_robots) {
    sub_robots.push_back(robots[i]);
  }
  std::vector<Task> sub_tasks;
  sub_tasks.reserve(open.size());
  for (const std::size_t k : open) {
    sub_tasks.push_back(tasks_[k]);
  }
  CostMatrix costs = weighted_costs(sub_robots, sub_tasks, config_.weights);
  for (std::size_t r = 0; r < free_robots.size(); ++r) {
    const auto& h = held[free_robots[r]];
    if (!h) {
      continue;
    }
    for (std::size_t c = 0; c < open.size(); ++c) {
      if (open[c] != *h) {
        costs(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) +=
            config_.switching_penalty;
      }
    }
  }
  Assignment solution;
  if (config_.method == AllocationMethod::kAuction) {
    const AuctionResult auction = solve_auction(costs, config_.auction);
    solution = auction.assignment;
    out.messages = auction.messages;
  } else {
    solution = solve_hungarian(costs);
  }

  for (std::size_t r = 0; r < free_robots.size(); ++r) {
    const std::size_t i = free_robots[r];
    if (const auto c = solution.robot_to_task[r]) {
      const std::size_t k = open[*c];
      out.robot_task[i] = tasks_[k].id;
      if (!held[i]) {
        ++out.newly_assigned;
      } else if (*held[i] != k) {
        ++out.reassigned;
      }
    }
  }
  // Report the true (penalty-free) cost of the final assignment.
  const CostMatrix all = weighted_costs(robots, tasks_, config_.weights);
  for (std::size_t i = 0; i < robots.size(); ++i) {
    if (const auto id = out.robot_task[i]) {
      for (std::size_t k = 0; k < tasks_.size(); ++k) {
        if (tasks_[k].id == *id) {
          out.cost += all(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k));
        }
      }
    }
  }
  return out;
}

}  // namespace multirobotcore::allocation
