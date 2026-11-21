#include "multirobotcore/allocation/types.hpp"

#include <cmath>
#include <stdexcept>

namespace multirobotcore::allocation {
namespace {

Eigen::Index idx(std::size_t i) {
  return static_cast<Eigen::Index>(i);
}

double route_length(const Robot& r, const Task& t) {
  double d = (t.position - r.position).norm();
  if (t.dropoff) {
    d += (*t.dropoff - t.position).norm();
  }
  return d;
}

}  // namespace

std::size_t Assignment::num_assigned() const noexcept {
  std::size_t count = 0;
  for (const auto& t : robot_to_task) {
    if (t) {
      ++count;
    }
  }
  return count;
}

bool Assignment::is_consistent() const noexcept {
  for (std::size_t r = 0; r < robot_to_task.size(); ++r) {
    const auto& t = robot_to_task[r];
    if (t && (*t >= task_to_robot.size() || task_to_robot[*t] != r)) {
      return false;
    }
  }
  for (std::size_t t = 0; t < task_to_robot.size(); ++t) {
    const auto& r = task_to_robot[t];
    if (r && (*r >= robot_to_task.size() || robot_to_task[*r] != t)) {
      return false;
    }
  }
  return true;
}

Assignment make_assignment(const CostMatrix& costs,
                           const std::vector<std::optional<std::size_t>>& robot_to_task) {
  if (robot_to_task.size() != static_cast<std::size_t>(costs.rows())) {
    throw std::invalid_argument("make_assignment: one entry per robot required");
  }
  Assignment a;
  a.robot_to_task = robot_to_task;
  a.task_to_robot.assign(static_cast<std::size_t>(costs.cols()), std::nullopt);
  for (std::size_t r = 0; r < robot_to_task.size(); ++r) {
    if (const auto& t = robot_to_task[r]) {
      if (*t >= a.task_to_robot.size() || a.task_to_robot[*t]) {
        throw std::invalid_argument("make_assignment: task out of range or assigned twice");
      }
      a.task_to_robot[*t] = r;
    }
  }
  a.total_cost = assignment_cost(costs, a);
  return a;
}

double assignment_cost(const CostMatrix& costs, const Assignment& assignment) {
  double total = 0.0;
  for (std::size_t r = 0; r < assignment.robot_to_task.size(); ++r) {
    if (const auto& t = assignment.robot_to_task[r]) {
      total += costs(idx(r), idx(*t));
    }
  }
  return total;
}

void validate_costs(const CostMatrix& costs) {
  for (Eigen::Index i = 0; i < costs.rows(); ++i) {
    for (Eigen::Index j = 0; j < costs.cols(); ++j) {
      const double c = costs(i, j);
      if (std::isnan(c) || c == -std::numeric_limits<double>::infinity()) {
        throw std::invalid_argument("cost matrix contains NaN or -infinity");
      }
    }
  }
}

CostMatrix custom_costs(std::span<const Robot> robots, std::span<const Task> tasks,
                        const std::function<double(const Robot&, const Task&)>& cost) {
  CostMatrix c(idx(robots.size()), idx(tasks.size()));
  for (std::size_t r = 0; r < robots.size(); ++r) {
    for (std::size_t t = 0; t < tasks.size(); ++t) {
      c(idx(r), idx(t)) = cost(robots[r], tasks[t]);
    }
  }
  return c;
}

CostMatrix euclidean_costs(std::span<const Robot> robots, std::span<const Task> tasks) {
  return custom_costs(robots, tasks, route_length);
}

CostMatrix travel_time_costs(std::span<const Robot> robots, std::span<const Task> tasks) {
  return custom_costs(robots, tasks, [](const Robot& r, const Task& t) {
    if (!(r.speed > 0.0)) {
      return kInfeasible;
    }
    return route_length(r, t) / r.speed + t.service_time;
  });
}

CostMatrix energy_costs(std::span<const Robot> robots, std::span<const Task> tasks) {
  return custom_costs(robots, tasks, [](const Robot& r, const Task& t) {
    return r.energy_per_meter * route_length(r, t);
  });
}

CostMatrix weighted_costs(std::span<const Robot> robots, std::span<const Task> tasks,
                          const CostWeights& w) {
  return custom_costs(robots, tasks, [&w](const Robot& r, const Task& t) {
    if (w.enforce_capacity && t.demand > r.capacity) {
      return kInfeasible;
    }
    const double d = route_length(r, t);
    double c = w.distance * d + w.energy * r.energy_per_meter * d - w.priority * t.priority;
    if (w.time != 0.0) {
      if (!(r.speed > 0.0)) {
        return kInfeasible;
      }
      c += w.time * (d / r.speed + t.service_time);
    }
    return c;
  });
}

}  // namespace multirobotcore::allocation
