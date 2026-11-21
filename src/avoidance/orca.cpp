#include "multirobotcore/avoidance/orca.hpp"

#include "multirobotcore/math/numerics.hpp"

#include <cmath>
#include <stdexcept>

namespace multirobotcore::avoidance {
namespace {

/// Unit vector of w, or `fallback` (normalised) when w is negligible.
Vec2 unit_or(const Vec2& w, const Vec2& fallback) {
  const double n = w.norm();
  if (n > 1e-12) {
    return w / n;
  }
  const Vec2 f = math::safe_normalized(fallback);
  return f.squaredNorm() > 0.0 ? f : Vec2(1.0, 0.0);
}

}  // namespace

OrcaConstraint orca_constraint(const AgentState& self, const AgentState& other, double time_horizon,
                               double time_step, double responsibility) {
  if (!(time_horizon > 0.0) || !(time_step > 0.0)) {
    throw std::invalid_argument("orca_constraint: time horizon and time step must be positive");
  }
  const Vec2 p = other.position - self.position;
  const Vec2 v = self.velocity - other.velocity;
  const double radius = self.radius + other.radius;
  const double dist_sq = p.squaredNorm();

  OrcaConstraint out;
  Vec2 n = Vec2::Zero();
  if (dist_sq > radius * radius) {
    // w: from the centre of the cut-off disc D(p / tau, R / tau) to v.
    const Vec2 w = v - p / time_horizon;
    const double w_sq = w.squaredNorm();
    const double w_dot_p = w.dot(p);
    if (w_dot_p < 0.0 && w_dot_p * w_dot_p > radius * radius * w_sq) {
      // Closest boundary point lies on the cut-off arc.
      n = unit_or(w, -p);
      out.u = (radius / time_horizon - std::sqrt(w_sq)) * n;
    } else {
      // Closest boundary point lies on one of the cone legs.
      const double leg = std::sqrt(dist_sq - radius * radius);
      Vec2 d;
      if (math::cross(p, w) > 0.0) {
        d = Vec2(p.x() * leg - p.y() * radius, p.x() * radius + p.y() * leg) / dist_sq;
        n = math::perp(d);  // outward normal of the left leg
      } else {
        d = Vec2(p.x() * leg + p.y() * radius, -p.x() * radius + p.y() * leg) / dist_sq;
        n = -math::perp(d);  // outward normal of the right leg
      }
      out.u = v.dot(d) * d - v;
    }
  } else {
    // Already overlapping: use the VO for one time step so that the agents
    // separate as quickly as possible.
    out.colliding = true;
    const Vec2 w = v - p / time_step;
    n = unit_or(w, -p);
    out.u = (radius / time_step - w.norm()) * n;
  }
  out.half_plane = HalfPlane{n, n.dot(self.velocity + responsibility * out.u)};
  return out;
}

OrcaVelocity orca_velocity(const AgentState& self, const Vec2& preferred, double max_speed,
                           std::span<const AgentState> neighbors,
                           std::span<const AgentState> static_obstacles, const OrcaParams& params) {
  OrcaVelocity out;
  out.constraints.reserve(neighbors.size() + static_obstacles.size());
  for (const AgentState& obstacle : static_obstacles) {
    out.constraints.push_back(
        orca_constraint(self, obstacle, params.obstacle_time_horizon, params.time_step, 1.0)
            .half_plane);
  }
  const std::size_t num_hard = out.constraints.size();
  for (const AgentState& other : neighbors) {
    out.constraints.push_back(
        orca_constraint(self, other, params.time_horizon, params.time_step, 0.5).half_plane);
  }
  const VelocityLpResult lp = solve_velocity_lp(out.constraints, num_hard, preferred, max_speed);
  out.velocity = lp.velocity;
  out.feasible = lp.feasible;
  out.max_violation = lp.max_violation;
  return out;
}

}  // namespace multirobotcore::avoidance
