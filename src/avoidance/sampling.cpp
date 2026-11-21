#include "multirobotcore/avoidance/sampling.hpp"

#include "multirobotcore/math/numerics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace multirobotcore::avoidance {
namespace {

std::vector<Vec2> candidate_velocities(const Vec2& preferred, double max_speed,
                                       const SamplingParams& params) {
  std::vector<Vec2> candidates;
  candidates.reserve(2 + params.speed_rings * params.angle_samples);
  candidates.push_back(math::clamp_norm(preferred, max_speed));
  candidates.emplace_back(Vec2::Zero());
  // Polar grid aligned with the preferred direction so that the sample set is
  // invariant under rotations of the scenario.
  const double base =
      preferred.squaredNorm() > 0.0 ? std::atan2(preferred.y(), preferred.x()) : 0.0;
  for (std::size_t ring = 1; ring <= params.speed_rings; ++ring) {
    const double speed =
        max_speed * static_cast<double>(ring) / static_cast<double>(params.speed_rings);
    for (std::size_t k = 0; k < params.angle_samples; ++k) {
      const double angle =
          base + math::kTwoPi * static_cast<double>(k) / static_cast<double>(params.angle_samples);
      candidates.emplace_back(speed * std::cos(angle), speed * std::sin(angle));
    }
  }
  return candidates;
}

}  // namespace

SampledVelocity sample_velocity(const AgentState& self, const Vec2& preferred, double max_speed,
                                std::span<const AgentState> neighbors,
                                std::span<const AgentState> static_obstacles, SamplingRule rule,
                                const SamplingParams& params) {
  if (params.speed_rings == 0 || params.angle_samples == 0) {
    throw std::invalid_argument("sample_velocity: at least one ring and one direction required");
  }
  SampledVelocity best;
  best.penalty = std::numeric_limits<double>::infinity();
  for (const Vec2& v : candidate_velocities(preferred, max_speed, params)) {
    double tc = kNever;
    for (const AgentState& other : neighbors) {
      const Vec2 relative = rule == SamplingRule::kReciprocalVelocityObstacle
                                ? Vec2(2.0 * v - self.velocity - other.velocity)
                                : Vec2(v - other.velocity);
      double t = effective_time_to_collision(other.position - self.position, relative,
                                             self.radius + other.radius);
      if (t > params.time_horizon) {
        t = kNever;
      }
      tc = std::min(tc, t);
    }
    for (const AgentState& obstacle : static_obstacles) {
      double t = effective_time_to_collision(obstacle.position - self.position,
                                             v - obstacle.velocity, self.radius + obstacle.radius);
      if (t > params.obstacle_time_horizon) {
        t = kNever;
      }
      tc = std::min(tc, t);
    }
    // w / tc with tc = 0 is an unbounded penalty; a tiny floor keeps the
    // comparison finite and still ranks colliding candidates last.
    const double collision_term = tc == kNever ? 0.0 : params.penalty_weight / std::max(tc, 1e-9);
    const double penalty = collision_term + (preferred - v).norm();
    if (penalty < best.penalty) {
      best = SampledVelocity{v, penalty, tc, tc == kNever};
    }
  }
  return best;
}

}  // namespace multirobotcore::avoidance
