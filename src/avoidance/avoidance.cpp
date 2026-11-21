#include "multirobotcore/avoidance/avoidance.hpp"

#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

namespace multirobotcore::avoidance {

std::string_view to_string(AvoidanceMethod method) noexcept {
  switch (method) {
    case AvoidanceMethod::kNone:
      return "none";
    case AvoidanceMethod::kVelocityObstacle:
      return "vo";
    case AvoidanceMethod::kReciprocalVelocityObstacle:
      return "rvo";
    case AvoidanceMethod::kOrca:
      return "orca";
  }
  return "unknown";
}

AvoidanceMethod parse_method(std::string_view name) {
  if (name == "none") {
    return AvoidanceMethod::kNone;
  }
  if (name == "vo") {
    return AvoidanceMethod::kVelocityObstacle;
  }
  if (name == "rvo") {
    return AvoidanceMethod::kReciprocalVelocityObstacle;
  }
  if (name == "orca") {
    return AvoidanceMethod::kOrca;
  }
  throw std::invalid_argument("unknown avoidance method: " + std::string(name));
}

std::vector<math::Circle> obstacle_discs(const math::ObstacleSet& obstacles, double max_spacing) {
  if (!(max_spacing > 0.0)) {
    throw std::invalid_argument("obstacle_discs: spacing must be positive");
  }
  std::vector<math::Circle> discs(obstacles.circles.begin(), obstacles.circles.end());
  for (const math::AABB& box : obstacles.boxes) {
    const auto nx = static_cast<std::size_t>(std::max(1.0, std::ceil(box.width() / max_spacing)));
    const auto ny = static_cast<std::size_t>(std::max(1.0, std::ceil(box.height() / max_spacing)));
    const double cw = box.width() / static_cast<double>(nx);
    const double ch = box.height() / static_cast<double>(ny);
    const double radius = 0.5 * std::hypot(cw, ch);  // circumscribes one cell
    for (std::size_t ix = 0; ix < nx; ++ix) {
      for (std::size_t iy = 0; iy < ny; ++iy) {
        // Interior cells are never reached before boundary cells.
        const bool boundary = ix == 0 || iy == 0 || ix + 1 == nx || iy + 1 == ny;
        if (!boundary) {
          continue;
        }
        const Vec2 center = box.min + Vec2((static_cast<double>(ix) + 0.5) * cw,
                                           (static_cast<double>(iy) + 0.5) * ch);
        discs.push_back(math::Circle{center, radius});
      }
    }
  }
  return discs;
}

Vec2 preferred_velocity(const Vec2& position, const Vec2& goal, double max_speed,
                        double slowdown_distance) noexcept {
  const Vec2 to_goal = goal - position;
  const double distance = to_goal.norm();
  if (distance <= 1e-12) {
    return Vec2::Zero();
  }
  const double speed =
      slowdown_distance > 0.0 ? max_speed * std::min(1.0, distance / slowdown_distance) : max_speed;
  return to_goal / distance * speed;
}

CollisionAvoider::CollisionAvoider(AvoidanceConfig config, const math::ObstacleSet& obstacles)
    : config_(config) {
  if (!(config_.time_horizon > 0.0) || !(config_.obstacle_time_horizon > 0.0) ||
      !(config_.time_step > 0.0) || !(config_.neighbor_distance > 0.0)) {
    throw std::invalid_argument("CollisionAvoider: horizons, time step and range must be positive");
  }
  set_obstacles(obstacles);
}

void CollisionAvoider::set_obstacles(const math::ObstacleSet& obstacles) {
  discs_ = obstacle_discs(obstacles, config_.obstacle_disc_spacing);
}

CollisionAvoider::Neighborhood CollisionAvoider::neighborhood(
    std::size_t i, std::span<const AgentState> agents, const std::vector<bool>& active) const {
  Neighborhood hood;
  const AgentState& self = agents[i];
  const double range_sq = config_.neighbor_distance * config_.neighbor_distance;

  // Closest robots first; ties broken by index for determinism.
  std::vector<std::pair<double, std::size_t>> candidates;
  for (std::size_t j = 0; j < agents.size(); ++j) {
    if (j == i) {
      continue;
    }
    const double d_sq = (agents[j].position - self.position).squaredNorm();
    if (d_sq <= range_sq) {
      candidates.emplace_back(d_sq, j);
    }
  }
  std::sort(candidates.begin(), candidates.end());
  std::size_t taken = 0;
  for (const auto& [d_sq, j] : candidates) {
    AgentState other = agents[j];
    other.radius += config_.safety_margin;
    const bool is_active = active.empty() || active[j];
    if (!is_active) {
      other.velocity.setZero();
      hood.obstacles.push_back(other);
      hood.obstacle_ids.push_back(j);
      hood.obstacle_is_robot.push_back(true);
    } else if (taken < config_.max_neighbors) {
      hood.agents.push_back(other);
      hood.agent_ids.push_back(j);
      ++taken;
    }
  }
  for (std::size_t k = 0; k < discs_.size(); ++k) {
    const math::Circle& c = discs_[k];
    const double surface = (c.center - self.position).norm() - c.radius;
    if (surface <= config_.neighbor_distance) {
      hood.obstacles.push_back(AgentState{c.center, Vec2::Zero(), c.radius});
      hood.obstacle_ids.push_back(k);
      hood.obstacle_is_robot.push_back(false);
    }
  }
  return hood;
}

Vec2 CollisionAvoider::select(const AgentState& self, const Vec2& preferred, double max_speed,
                              const Neighborhood& hood, bool* feasible) const {
  *feasible = true;
  switch (config_.method) {
    case AvoidanceMethod::kNone:
      return math::clamp_norm(preferred, max_speed);
    case AvoidanceMethod::kVelocityObstacle:
    case AvoidanceMethod::kReciprocalVelocityObstacle: {
      const SamplingParams params{config_.time_horizon, config_.obstacle_time_horizon,
                                  config_.speed_rings, config_.angle_samples,
                                  config_.penalty_weight};
      const SamplingRule rule = config_.method == AvoidanceMethod::kVelocityObstacle
                                    ? SamplingRule::kVelocityObstacle
                                    : SamplingRule::kReciprocalVelocityObstacle;
      const SampledVelocity s =
          sample_velocity(self, preferred, max_speed, hood.agents, hood.obstacles, rule, params);
      *feasible = s.admissible;
      return s.velocity;
    }
    case AvoidanceMethod::kOrca: {
      const OrcaParams params{config_.time_horizon, config_.obstacle_time_horizon,
                              config_.time_step};
      const OrcaVelocity o =
          orca_velocity(self, preferred, max_speed, hood.agents, hood.obstacles, params);
      *feasible = o.feasible;
      return o.velocity;
    }
  }
  return Vec2::Zero();
}

std::vector<Vec2> CollisionAvoider::compute(std::span<const AgentState> agents,
                                            std::span<const Vec2> preferred,
                                            std::span<const double> max_speeds,
                                            const std::vector<bool>& active) const {
  const std::size_t n = agents.size();
  if (preferred.size() != n || max_speeds.size() != n || (!active.empty() && active.size() != n)) {
    throw std::invalid_argument("CollisionAvoider::compute: size mismatch");
  }
  std::vector<Vec2> out(n, Vec2::Zero());
  last_infeasible_ = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (!active.empty() && !active[i]) {
      continue;
    }
    AgentState self = agents[i];
    self.radius += config_.safety_margin;
    bool feasible = true;
    out[i] = select(self, preferred[i], max_speeds[i], neighborhood(i, agents, active), &feasible);
    if (!feasible && config_.method == AvoidanceMethod::kOrca) {
      ++last_infeasible_;
    }
  }
  return out;
}

AgentDiagnostics CollisionAvoider::diagnose(std::size_t i, std::span<const AgentState> agents,
                                            std::span<const Vec2> preferred,
                                            std::span<const double> max_speeds,
                                            const std::vector<bool>& active) const {
  if (i >= agents.size() || preferred.size() != agents.size() ||
      max_speeds.size() != agents.size()) {
    throw std::invalid_argument("CollisionAvoider::diagnose: invalid index or size mismatch");
  }
  AgentState self = agents[i];
  self.radius += config_.safety_margin;
  const Neighborhood hood = neighborhood(i, agents, active);
  AgentDiagnostics d;
  d.preferred = preferred[i];
  d.selected = select(self, preferred[i], max_speeds[i], hood, &d.feasible);
  for (std::size_t k = 0; k < hood.agents.size(); ++k) {
    const AgentState& other = hood.agents[k];
    d.neighbors.push_back(NeighborDiagnostics{
        hood.agent_ids[k], true, false, velocity_obstacle(self, other, config_.time_horizon),
        reciprocal_velocity_obstacle(self, other, config_.time_horizon),
        orca_constraint(self, other, config_.time_horizon, config_.time_step, 0.5)});
  }
  for (std::size_t k = 0; k < hood.obstacles.size(); ++k) {
    const AgentState& other = hood.obstacles[k];
    const auto vo = velocity_obstacle(self, other, config_.obstacle_time_horizon);
    d.neighbors.push_back(NeighborDiagnostics{
        hood.obstacle_ids[k], hood.obstacle_is_robot[k], true, vo, vo,
        orca_constraint(self, other, config_.obstacle_time_horizon, config_.time_step, 1.0)});
  }
  return d;
}

}  // namespace multirobotcore::avoidance
