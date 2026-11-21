#include "multirobotcore/avoidance/velocity_obstacle.hpp"

#include <algorithm>
#include <cmath>

namespace multirobotcore::avoidance {

double time_to_collision(const Vec2& relative_position, const Vec2& relative_velocity,
                         double combined_radius) noexcept {
  const Vec2& p = relative_position;
  const Vec2& v = relative_velocity;
  const double c = p.squaredNorm() - combined_radius * combined_radius;
  if (c <= 0.0) {
    return 0.0;
  }
  // ||p - t v||^2 = R^2  <=>  (v.v) t^2 - 2 (p.v) t + c = 0
  const double a = v.squaredNorm();
  const double b = p.dot(v);
  if (a <= 0.0 || b <= 0.0) {
    return kNever;  // static or moving apart
  }
  const double discriminant = b * b - a * c;
  if (discriminant < 0.0) {
    return kNever;
  }
  return (b - std::sqrt(discriminant)) / a;
}

double effective_time_to_collision(const Vec2& relative_position, const Vec2& relative_velocity,
                                   double combined_radius) noexcept {
  if (relative_position.squaredNorm() <= combined_radius * combined_radius) {
    return relative_velocity.dot(relative_position) < 0.0 ? kNever : 0.0;
  }
  return time_to_collision(relative_position, relative_velocity, combined_radius);
}

double min_distance_within_horizon(const Vec2& relative_position, const Vec2& relative_velocity,
                                   double time_horizon) noexcept {
  const double a = relative_velocity.squaredNorm();
  double t = 0.0;
  if (a > 0.0) {
    t = std::clamp(relative_position.dot(relative_velocity) / a, 0.0, time_horizon);
  }
  return (relative_position - t * relative_velocity).norm();
}

VOMembership classify_relative_velocity(const Vec2& relative_position,
                                        const Vec2& relative_velocity, double combined_radius,
                                        double time_horizon, double tolerance) noexcept {
  const double d = min_distance_within_horizon(relative_position, relative_velocity, time_horizon);
  if (d < combined_radius - tolerance) {
    return VOMembership::kInside;
  }
  if (d <= combined_radius + tolerance) {
    return VOMembership::kBoundary;
  }
  return VOMembership::kOutside;
}

VOMembership VelocityObstacleCone::classify(const Vec2& velocity, double tolerance) const noexcept {
  return classify_relative_velocity(relative_position, velocity - apex, combined_radius,
                                    time_horizon, tolerance);
}

namespace {

VelocityObstacleCone make_cone(const Vec2& apex, const Vec2& p, double radius, double horizon) {
  VelocityObstacleCone cone;
  cone.apex = apex;
  cone.relative_position = p;
  cone.combined_radius = radius;
  cone.time_horizon = horizon;
  cone.cutoff_center = apex + p / horizon;
  cone.cutoff_radius = radius / horizon;
  const double dist_sq = p.squaredNorm();
  if (dist_sq <= radius * radius) {
    cone.overlapping = true;
    return cone;
  }
  // Legs: p rotated by +/- asin(R / |p|), normalised.
  const double leg = std::sqrt(dist_sq - radius * radius);
  cone.left_leg = Vec2(p.x() * leg - p.y() * radius, p.x() * radius + p.y() * leg) / dist_sq;
  cone.right_leg = Vec2(p.x() * leg + p.y() * radius, -p.x() * radius + p.y() * leg) / dist_sq;
  return cone;
}

}  // namespace

VelocityObstacleCone velocity_obstacle(const AgentState& self, const AgentState& other,
                                       double time_horizon) noexcept {
  return make_cone(other.velocity, other.position - self.position, self.radius + other.radius,
                   time_horizon);
}

VelocityObstacleCone reciprocal_velocity_obstacle(const AgentState& self, const AgentState& other,
                                                  double time_horizon) noexcept {
  return make_cone(0.5 * (self.velocity + other.velocity), other.position - self.position,
                   self.radius + other.radius, 2.0 * time_horizon);
}

}  // namespace multirobotcore::avoidance
